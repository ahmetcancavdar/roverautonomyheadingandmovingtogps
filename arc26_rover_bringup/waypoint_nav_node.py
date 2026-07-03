import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from sensor_msgs.msg import NavSatFix

from .gps_math import angle_error_deg, bearing_between_gps_deg, normalize_heading_deg


EARTH_RADIUS_M = 6371000.0


def distance_between_gps_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)

    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2.0) ** 2
    )

    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c


class WaypointNavNode(Node):
    def __init__(self):
        super().__init__("waypoint_nav_node")

        # Target waypoint
        self.declare_parameter("target_lat", 0.0)
        self.declare_parameter("target_lon", 0.0)

        # Dynamic goal radius:
        # RTK_FIXED q=4 -> fixed_goal_radius_m
        # RTK_FLOAT/SPS/valid non-fixed GPS -> non_fixed_goal_radius_m
        self.declare_parameter("fixed_goal_radius_m", 0.30)
        self.declare_parameter("non_fixed_goal_radius_m", 0.60)

        # Heading hysteresis:
        # TURN -> FORWARD when abs_error <= heading_tolerance_deg
        # FORWARD -> TURN when abs_error >= reacquire_heading_error_deg
        self.declare_parameter("heading_tolerance_deg", 5.0)
        self.declare_parameter("reacquire_heading_error_deg", 10.0)

        # Motor parameters
        self.declare_parameter("turn_pwm_fast", 75)
        self.declare_parameter("turn_pwm_slow", 45)
        self.declare_parameter("forward_pwm", 60)

        # If heading error is smaller than this, slow turn PWM is used
        self.declare_parameter("slow_turn_threshold_deg", 30.0)

        # Command publishing rate limit
        self.declare_parameter("command_interval_sec", 0.25)

        # Sensor timeout limits
        self.declare_parameter("gps_timeout_sec", 1.5)
        self.declare_parameter("heading_timeout_sec", 1.0)

        # Use only if left/right motor commands are reversed
        self.declare_parameter("invert_turn_direction", False)

        self.target_lat = float(self.get_parameter("target_lat").value)
        self.target_lon = float(self.get_parameter("target_lon").value)

        self.fixed_goal_radius_m = float(
            self.get_parameter("fixed_goal_radius_m").value
        )
        self.non_fixed_goal_radius_m = float(
            self.get_parameter("non_fixed_goal_radius_m").value
        )

        self.heading_tolerance_deg = float(
            self.get_parameter("heading_tolerance_deg").value
        )
        self.reacquire_heading_error_deg = float(
            self.get_parameter("reacquire_heading_error_deg").value
        )

        self.turn_pwm_fast = int(self.get_parameter("turn_pwm_fast").value)
        self.turn_pwm_slow = int(self.get_parameter("turn_pwm_slow").value)
        self.forward_pwm = int(self.get_parameter("forward_pwm").value)

        self.slow_turn_threshold_deg = float(
            self.get_parameter("slow_turn_threshold_deg").value
        )
        self.command_interval_sec = float(
            self.get_parameter("command_interval_sec").value
        )

        self.gps_timeout_sec = float(self.get_parameter("gps_timeout_sec").value)
        self.heading_timeout_sec = float(self.get_parameter("heading_timeout_sec").value)

        self.invert_turn_direction = bool(
            self.get_parameter("invert_turn_direction").value
        )

        self.latest_lat = None
        self.latest_lon = None
        self.latest_gps_time = 0.0

        self.latest_heading = None
        self.latest_heading_time = 0.0

        self.rtk_quality = None
        self.rtk_fix_name = "UNKNOWN"
        self.latest_rtk_status_time = 0.0

        self.goal_reached = False

        # Start by turning toward the target before moving forward
        self.nav_mode = "TURN"

        self.last_command_time = 0.0
        self.last_log_time = 0.0

        self.cmd_pub = self.create_publisher(String, "/cmd_drive", 10)

        self.target_heading_pub = self.create_publisher(
            Float32,
            "/navigation/target_heading_deg",
            10,
        )

        self.distance_pub = self.create_publisher(
            Float32,
            "/navigation/distance_to_target_m",
            10,
        )

        self.nav_status_pub = self.create_publisher(
            String,
            "/navigation/status",
            10,
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_callback,
            10,
        )

        self.heading_sub = self.create_subscription(
            Float32,
            "/compass/heading_deg",
            self.heading_callback,
            10,
        )

        self.rtk_status_sub = self.create_subscription(
            String,
            "/gps/rtk_status",
            self.rtk_status_callback,
            10,
        )

        self.control_timer = self.create_timer(0.10, self.control_loop)

        self.get_logger().info("waypoint_nav_node started.")
        self.get_logger().info(
            f"Target GPS: lat={self.target_lat:.8f}, "
            f"lon={self.target_lon:.8f}"
        )
        self.get_logger().info(
            f"Goal radius: RTK_FIXED={self.fixed_goal_radius_m:.2f} m, "
            f"non_fixed={self.non_fixed_goal_radius_m:.2f} m"
        )
        self.get_logger().info(
            f"Heading hysteresis: forward_threshold={self.heading_tolerance_deg:.2f} deg, "
            f"reacquire_threshold={self.reacquire_heading_error_deg:.2f} deg"
        )

    def gps_callback(self, msg: NavSatFix):
        self.latest_lat = float(msg.latitude)
        self.latest_lon = float(msg.longitude)
        self.latest_gps_time = time.time()

    def heading_callback(self, msg: Float32):
        self.latest_heading = float(msg.data)
        self.latest_heading_time = time.time()

    def rtk_status_callback(self, msg: String):
        self.latest_rtk_status_time = time.time()

        # Expected format:
        # q=4,fix=RTK_FIXED,lat=...,lon=...,alt=...,sats=...,hdop=...,age=...
        parts = msg.data.split(",")

        for part in parts:
            part = part.strip()

            if part.startswith("q="):
                self.rtk_quality = part.split("=", 1)[1]

            elif part.startswith("fix="):
                self.rtk_fix_name = part.split("=", 1)[1]

    def publish_cmd(self, cmd: str):
        now = time.time()

        if now - self.last_command_time < self.command_interval_sec:
            return

        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.last_command_time = now

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.nav_status_pub.publish(msg)

    def log_throttled(self, text: str, interval: float = 0.5):
        now = time.time()

        if now - self.last_log_time >= interval:
            self.get_logger().info(text)
            self.last_log_time = now

    def choose_turn_pwm(self, abs_error: float) -> int:
        if abs_error <= self.slow_turn_threshold_deg:
            return self.turn_pwm_slow

        return self.turn_pwm_fast

    def get_active_goal_radius_m(self) -> float:
        """
        RTK_FIXED varsa 30 cm hedef yarıçapı kullanılır.
        RTK_FIXED yoksa ama GPS konumu varsa 60 cm hedef yarıçapı kullanılır.
        """
        if self.rtk_quality == "4":
            return self.fixed_goal_radius_m

        return self.non_fixed_goal_radius_m

    def stop(self, reason: str):
        self.nav_mode = "TURN"
        self.publish_cmd("MOTOR:STOP")
        self.publish_status(f"STOP,{reason}")
        self.log_throttled(f"STOP: {reason}")

    def control_loop(self):
        now = time.time()

        if self.goal_reached:
            self.stop("GOAL_REACHED")
            return

        if abs(self.target_lat) < 0.000001 and abs(self.target_lon) < 0.000001:
            self.stop("TARGET_NOT_SET")
            return

        # GPS hiç gelmediyse dur
        if self.latest_lat is None or self.latest_lon is None:
            self.stop("NO_GPS")
            return

        # GPS önceden gelmiş ama uzun süredir yenilenmemişse dur
        if now - self.latest_gps_time > self.gps_timeout_sec:
            self.stop("GPS_TIMEOUT")
            return

        # Heading hiç gelmediyse dur
        if self.latest_heading is None:
            self.stop("NO_HEADING")
            return

        # Heading önceden gelmiş ama uzun süredir yenilenmemişse dur
        if now - self.latest_heading_time > self.heading_timeout_sec:
            self.stop("HEADING_TIMEOUT")
            return

        distance_m = distance_between_gps_m(
            self.latest_lat,
            self.latest_lon,
            self.target_lat,
            self.target_lon,
        )

        distance_msg = Float32()
        distance_msg.data = float(distance_m)
        self.distance_pub.publish(distance_msg)

        active_goal_radius_m = self.get_active_goal_radius_m()

        if distance_m <= active_goal_radius_m:
            self.goal_reached = True
            self.stop(
                f"ARRIVED distance={distance_m:.3f}m "
                f"radius={active_goal_radius_m:.2f}m "
                f"rtk_q={self.rtk_quality} "
                f"rtk_fix={self.rtk_fix_name}"
            )
            return

        target_heading = normalize_heading_deg(
            bearing_between_gps_deg(
                self.latest_lat,
                self.latest_lon,
                self.target_lat,
                self.target_lon,
            )
        )

        target_msg = Float32()
        target_msg.data = float(target_heading)
        self.target_heading_pub.publish(target_msg)

        current_heading = self.latest_heading
        error = angle_error_deg(target_heading, current_heading)
        abs_error = abs(error)

        # Heading control with hysteresis:
        #
        # TURN mode:
        #   Rover rotates until the heading error becomes small enough.
        #
        # FORWARD mode:
        #   Rover keeps moving forward even if the error slightly increases.
        #   It only re-enters TURN mode when the error grows clearly.
        #
        # This prevents rapid TURN/FORWARD oscillation around the threshold.
        if self.nav_mode == "TURN":
            if abs_error <= self.heading_tolerance_deg:
                self.nav_mode = "FORWARD"

        elif self.nav_mode == "FORWARD":
            if abs_error >= self.reacquire_heading_error_deg:
                self.nav_mode = "TURN"

        if self.nav_mode == "TURN":
            pwm = self.choose_turn_pwm(abs_error)

            turn_right = error > 0.0

            if self.invert_turn_direction:
                turn_right = not turn_right

            if turn_right:
                cmd = f"MOTOR:RIGHT:{pwm}"
            else:
                cmd = f"MOTOR:LEFT:{pwm}"

            self.publish_cmd(cmd)
            mode = "TURN"

        else:
            cmd = f"MOTOR:FWD:{self.forward_pwm}"
            self.publish_cmd(cmd)
            mode = "FORWARD"

        status = (
            f"{mode},"
            f"nav_mode={self.nav_mode},"
            f"lat={self.latest_lat:.8f},"
            f"lon={self.latest_lon:.8f},"
            f"target_lat={self.target_lat:.8f},"
            f"target_lon={self.target_lon:.8f},"
            f"distance_m={distance_m:.3f},"
            f"active_goal_radius_m={active_goal_radius_m:.2f},"
            f"heading={current_heading:.2f},"
            f"target_heading={target_heading:.2f},"
            f"error={error:.2f},"
            f"abs_error={abs_error:.2f},"
            f"cmd={cmd},"
            f"rtk_q={self.rtk_quality},"
            f"rtk_fix={self.rtk_fix_name}"
        )

        self.publish_status(status)
        self.log_throttled(status)

    def destroy_node(self):
        try:
            msg = String()
            msg.data = "MOTOR:STOP"
            self.cmd_pub.publish(msg)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = WaypointNavNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
