import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .gps_math import angle_error_deg, normalize_heading_deg


class HeadingTurnNode(Node):
    def __init__(self):
        super().__init__("heading_turn_node")

        self.declare_parameter("target_heading", 90.0)
        self.declare_parameter("turn_pwm_fast", 90)
        self.declare_parameter("turn_pwm_slow", 65)
        self.declare_parameter("heading_tolerance_deg", 7.0)
        self.declare_parameter("slow_turn_threshold_deg", 25.0)
        self.declare_parameter("command_interval_sec", 0.15)
        self.declare_parameter("heading_timeout_sec", 1.0)
        self.declare_parameter("invert_turn_direction", False)

        self.target_heading = normalize_heading_deg(float(self.get_parameter("target_heading").value))
        self.turn_pwm_fast = int(self.get_parameter("turn_pwm_fast").value)
        self.turn_pwm_slow = int(self.get_parameter("turn_pwm_slow").value)
        self.heading_tolerance_deg = float(self.get_parameter("heading_tolerance_deg").value)
        self.slow_turn_threshold_deg = float(self.get_parameter("slow_turn_threshold_deg").value)
        self.command_interval_sec = float(self.get_parameter("command_interval_sec").value)
        self.heading_timeout_sec = float(self.get_parameter("heading_timeout_sec").value)
        self.invert_turn_direction = bool(self.get_parameter("invert_turn_direction").value)

        self.latest_heading = None
        self.latest_heading_time = 0.0
        self.last_command_time = 0.0
        self.last_log_time = 0.0

        self.cmd_pub = self.create_publisher(String, "/cmd_drive", 10)

        self.heading_sub = self.create_subscription(
            Float32,
            "/compass/heading_deg",
            self.heading_callback,
            10,
        )

        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f"HeadingTurnNode started. target={self.target_heading:.2f}")

    def heading_callback(self, msg: Float32):
        self.latest_heading = float(msg.data)
        self.latest_heading_time = time.time()

    def choose_pwm(self, abs_error: float) -> int:
        if abs_error <= self.slow_turn_threshold_deg:
            return self.turn_pwm_slow
        return self.turn_pwm_fast

    def publish_cmd(self, cmd: str):
        now = time.time()

        if now - self.last_command_time < self.command_interval_sec:
            return

        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.last_command_time = now

    def log_throttled(self, text: str):
        now = time.time()
        if now - self.last_log_time >= 0.3:
            self.get_logger().info(text)
            self.last_log_time = now

    def control_loop(self):
        now = time.time()

        if self.latest_heading is None or now - self.latest_heading_time > self.heading_timeout_sec:
            self.publish_cmd("MOTOR:STOP")
            self.log_throttled("No heading. MOTOR:STOP")
            return

        heading = self.latest_heading
        error = angle_error_deg(self.target_heading, heading)
        abs_error = abs(error)

        if abs_error <= self.heading_tolerance_deg:
            self.publish_cmd("MOTOR:STOP")
            self.log_throttled(
                f"DONE heading={heading:.2f} target={self.target_heading:.2f} error={error:.2f}"
            )
            return

        pwm = self.choose_pwm(abs_error)
        turn_right = error > 0.0

        if self.invert_turn_direction:
            turn_right = not turn_right

        if turn_right:
            cmd = f"MOTOR:RIGHT:{pwm}"
        else:
            cmd = f"MOTOR:LEFT:{pwm}"

        self.publish_cmd(cmd)

        self.log_throttled(
            f"AUTO heading={heading:.2f} target={self.target_heading:.2f} "
            f"error={error:.2f} cmd={cmd}"
        )

    def destroy_node(self):
        msg = String()
        msg.data = "MOTOR:STOP"
        self.cmd_pub.publish(msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = HeadingTurnNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
