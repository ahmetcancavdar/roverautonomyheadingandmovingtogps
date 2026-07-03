import time
import threading
from collections import Counter

import serial
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus


def crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
            crc &= 0xFFFFFF
    return crc


def rtcm_message_type(frame: bytes) -> int:
    payload = frame[3:-3]
    if len(payload) < 2:
        return -1
    return (payload[0] << 4) | (payload[1] >> 4)


def extract_rtcm3_frames(buffer: bytearray):
    frames = []

    while True:
        start = buffer.find(b"\xD3")

        if start < 0:
            return bytearray(), frames

        if len(buffer) - start < 3:
            return bytearray(buffer[start:]), frames

        if buffer[start + 1] & 0xFC:
            buffer = buffer[start + 1:]
            continue

        length = ((buffer[start + 1] & 0x03) << 8) | buffer[start + 2]
        total_len = 3 + length + 3

        if len(buffer) - start < total_len:
            return bytearray(buffer[start:]), frames

        frame = bytes(buffer[start:start + total_len])

        received_crc = (
            (frame[-3] << 16)
            | (frame[-2] << 8)
            | frame[-1]
        )

        calculated_crc = crc24q(frame[:-3])

        if received_crc == calculated_crc:
            frames.append(frame)
            buffer = buffer[start + total_len:]
        else:
            buffer = buffer[start + 1:]


def nmea2dec(coord: str, direction: str) -> float:
    if not coord:
        raise ValueError("Empty coordinate")

    dot = coord.index(".")
    deg = float(coord[:dot - 2])
    mins = float(coord[dot - 2:])
    dec = deg + mins / 60.0

    if direction in ("S", "W"):
        dec = -dec

    return dec


def parse_gga(line: str):
    if "GGA," not in line:
        return None

    parts = line.split(",")

    if len(parts) < 10:
        return None

    try:
        utc_time = parts[1]
        lat_raw = parts[2]
        lat_dir = parts[3]
        lon_raw = parts[4]
        lon_dir = parts[5]
        quality = parts[6]
        satellites = parts[7]
        hdop = parts[8]
        altitude = parts[9]
        diff_age = parts[13] if len(parts) > 13 else ""

        lat = None
        lon = None

        if quality != "0" and lat_raw and lon_raw:
            lat = nmea2dec(lat_raw, lat_dir)
            lon = nmea2dec(lon_raw, lon_dir)

        return {
            "utc_time": utc_time,
            "lat": lat,
            "lon": lon,
            "quality": quality,
            "satellites": satellites,
            "hdop": hdop,
            "altitude": altitude,
            "diff_age": diff_age,
            "raw": line.strip(),
        }

    except Exception:
        return None


def fix_name(quality: str) -> str:
    names = {
        "0": "INVALID",
        "1": "SPS",
        "2": "DGPS",
        "4": "RTK_FIXED",
        "5": "RTK_FLOAT",
        "6": "DEAD_RECKONING",
    }
    return names.get(quality, "UNKNOWN")


class GpsRtkNode(Node):
    def __init__(self):
        super().__init__("gps_rtk_node")

        self.declare_parameter("gps_port", "/dev/ttyUSB1")
        self.declare_parameter("radio_port", "/dev/ttyUSB0")
        self.declare_parameter("gps_baud", 460800)
        self.declare_parameter("radio_baud", 57600)
        self.declare_parameter("serial_timeout", 0.05)
        self.declare_parameter("print_interval_sec", 1.0)

        self.gps_port = str(self.get_parameter("gps_port").value)
        self.radio_port = str(self.get_parameter("radio_port").value)
        self.gps_baud = int(self.get_parameter("gps_baud").value)
        self.radio_baud = int(self.get_parameter("radio_baud").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout").value)
        self.print_interval_sec = float(self.get_parameter("print_interval_sec").value)

        self.fix_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.status_pub = self.create_publisher(String, "/gps/rtk_status", 10)
        self.rtcm_pub = self.create_publisher(String, "/gps/rtcm_status", 10)

        self.gps_ser = None
        self.radio_ser = None
        self.running = False

        self.last_print_time = 0.0

        self.connect_serial()

        self.running = True

        self.radio_thread = threading.Thread(target=self.radio_to_gps_rtcm_loop, daemon=True)
        self.gps_thread = threading.Thread(target=self.gps_gga_loop, daemon=True)

        self.radio_thread.start()
        self.gps_thread.start()

        self.get_logger().info("gps_rtk_node started.")

    def connect_serial(self):
        self.get_logger().info(f"Opening rover GPS: {self.gps_port} @ {self.gps_baud}")
        self.get_logger().info(f"Opening rover radio: {self.radio_port} @ {self.radio_baud}")

        self.gps_ser = serial.Serial(
            self.gps_port,
            self.gps_baud,
            timeout=self.serial_timeout,
            write_timeout=1,
        )

        self.radio_ser = serial.Serial(
            self.radio_port,
            self.radio_baud,
            timeout=self.serial_timeout,
            write_timeout=1,
        )

        self.gps_ser.reset_input_buffer()
        self.gps_ser.reset_output_buffer()
        self.radio_ser.reset_input_buffer()
        self.radio_ser.reset_output_buffer()

        self.get_logger().info("GPS and radio serial ports opened.")

    def radio_to_gps_rtcm_loop(self):
        buffer = bytearray()
        last_report = time.time()
        byte_count = 0
        frame_count = 0
        msg_counter = Counter()

        while self.running:
            try:
                data = self.radio_ser.read(self.radio_ser.in_waiting or 1)

                if data:
                    buffer.extend(data)
                    buffer, frames = extract_rtcm3_frames(buffer)

                    for frame in frames:
                        self.gps_ser.write(frame)
                        self.gps_ser.flush()

                        byte_count += len(frame)
                        frame_count += 1
                        msg_counter[rtcm_message_type(frame)] += 1

                now = time.time()

                if now - last_report >= 1.0:
                    if frame_count > 0:
                        top_msgs = ", ".join(
                            f"{msg}:{cnt}" for msg, cnt in msg_counter.most_common(6)
                        )
                        text = (
                            f"rtcm_bytes_per_sec={byte_count},"
                            f"rtcm_frames_per_sec={frame_count},"
                            f"msg={top_msgs}"
                        )
                    else:
                        text = "rtcm_bytes_per_sec=0,rtcm_frames_per_sec=0,msg=NO_RTCM"

                    msg = String()
                    msg.data = text
                    self.rtcm_pub.publish(msg)

                    byte_count = 0
                    frame_count = 0
                    msg_counter.clear()
                    last_report = now

            except Exception as exc:
                self.get_logger().error(f"RTCM radio->GPS error: {exc}")
                time.sleep(0.2)

    def gps_gga_loop(self):
        while self.running:
            try:
                raw_line = self.gps_ser.readline()

                if not raw_line:
                    continue

                line = raw_line.decode("ascii", errors="ignore").strip()

                if not line:
                    continue

                gga = parse_gga(line)

                if gga is None:
                    continue

                self.publish_gga(gga)

            except Exception as exc:
                self.get_logger().error(f"GPS GGA monitor error: {exc}")
                time.sleep(0.2)

    def publish_gga(self, gga: dict):
        quality = gga["quality"]
        fix = fix_name(quality)
        lat = gga["lat"]
        lon = gga["lon"]

        sats = gga["satellites"] or "0"
        hdop = gga["hdop"] or ""
        altitude = gga["altitude"] or "0"
        diff_age = gga["diff_age"] or ""

        status_text = (
            f"q={quality},"
            f"fix={fix},"
            f"lat={lat if lat is not None else ''},"
            f"lon={lon if lon is not None else ''},"
            f"alt={altitude},"
            f"sats={sats},"
            f"hdop={hdop},"
            f"age={diff_age}"
        )

        status_msg = String()
        status_msg.data = status_text
        self.status_pub.publish(status_msg)

        if lat is None or lon is None:
            return

        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = "gps"

        fix_msg.latitude = float(lat)
        fix_msg.longitude = float(lon)

        try:
            fix_msg.altitude = float(altitude)
        except Exception:
            fix_msg.altitude = 0.0

        fix_msg.status.service = NavSatStatus.SERVICE_GPS

        if quality == "0":
            fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
        elif quality in ("4", "5"):
            fix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        else:
            fix_msg.status.status = NavSatStatus.STATUS_FIX

        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(fix_msg)

        now = time.time()
        if now - self.last_print_time >= self.print_interval_sec:
            self.get_logger().info(
                f"GPS fix={fix} q={quality} lat={lat:.8f} lon={lon:.8f} "
                f"alt={altitude} sats={sats} hdop={hdop} age={diff_age}"
            )
            self.last_print_time = now

    def destroy_node(self):
        self.running = False
        time.sleep(0.2)

        try:
            if self.gps_ser is not None:
                self.gps_ser.close()
        except Exception:
            pass

        try:
            if self.radio_ser is not None:
                self.radio_ser.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsRtkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()