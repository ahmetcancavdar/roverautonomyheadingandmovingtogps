import time
import threading
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .gps_math import normalize_heading_deg


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__("arduino_bridge_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("heading_offset_deg", 0.0)
        self.declare_parameter("plane", "XY")

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.heading_offset_deg = float(self.get_parameter("heading_offset_deg").value)
        self.plane = str(self.get_parameter("plane").value)

        self.ser = None
        self.running = False
        self.read_thread = None

        self.heading_pub = self.create_publisher(Float32, "/compass/heading_deg", 10)
        self.raw_pub = self.create_publisher(String, "/arduino/raw_line", 10)
        self.status_pub = self.create_publisher(String, "/arduino/status", 10)

        self.cmd_sub = self.create_subscription(
            String,
            "/cmd_drive",
            self.cmd_drive_callback,
            10,
        )

        self.connect_serial()

        self.status_timer = self.create_timer(1.0, self.publish_status)

    def connect_serial(self):
        self.get_logger().info(f"Opening Arduino serial: {self.port} @ {self.baud}")

        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2.0)

        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        self.send_command("TELEM:ON")
        self.send_command(f"PLANE:{self.plane}")

        self.get_logger().info("Arduino bridge started.")

    def send_command(self, cmd: str):
        if self.ser is None:
            return

        line = cmd.strip() + "\n"
        self.ser.write(line.encode("ascii", errors="ignore"))

    def cmd_drive_callback(self, msg: String):
        cmd = msg.data.strip()

        allowed = (
            cmd == "MOTOR:STOP"
            or cmd.startswith("MOTOR:FWD:")
            or cmd.startswith("MOTOR:BACK:")
            or cmd.startswith("MOTOR:LEFT:")
            or cmd.startswith("MOTOR:RIGHT:")
        )

        if not allowed:
            self.get_logger().warn(f"Rejected drive command: {cmd}")
            return

        self.send_command(cmd)

    def read_loop(self):
        while self.running:
            try:
                raw = self.ser.readline()

                if not raw:
                    continue

                line = raw.decode("ascii", errors="ignore").strip()

                if not line:
                    continue

                raw_msg = String()
                raw_msg.data = line
                self.raw_pub.publish(raw_msg)

                self.parse_arduino_line(line)

            except Exception as exc:
                self.get_logger().error(f"Serial read error: {exc}")
                time.sleep(0.2)

    def parse_arduino_line(self, line: str):
        if line.startswith(("BOOT", "ACK", "WARN", "ERR", "STATUS", "PONG")):
            msg = String()
            msg.data = line
            self.status_pub.publish(msg)
            return

        # Arduino format:
        # MAG,time_ms,heading,rawX,rawY,rawZ,calX,calY,calZ,plane,offset,motor_mode,pwm
        if not line.startswith("MAG,"):
            return

        parts = line.split(",")

        if len(parts) < 13:
            return

        try:
            heading = float(parts[2])
        except ValueError:
            return

        heading = normalize_heading_deg(heading + self.heading_offset_deg)

        msg = Float32()
        msg.data = float(heading)
        self.heading_pub.publish(msg)

    def publish_status(self):
        msg = String()
        msg.data = f"arduino_bridge_alive port={self.port} baud={self.baud}"
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.running = False

        try:
            self.send_command("MOTOR:STOP")
            time.sleep(0.1)
        except Exception:
            pass

        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ArduinoBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
