#!/usr/bin/env python3

import math
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Empty, Float64, Int32, String


GPS_RELIABLE = "GPS_RELIABLE"
GPS_DEGRADED_GRACE = "GPS_DEGRADED_GRACE"
GPS_UNRELIABLE = "GPS_UNRELIABLE"
VISION_CLEAR = 0
VISION_VALID_DEPTH = 1
VISION_DEPTH_UNKNOWN = 2


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(quaternion):
    """Extract planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def horizontal_gps_sigma(msg):
    """Return conservative ENU horizontal sigma, or infinity if unknown."""
    covariance = msg.position_covariance
    if len(covariance) != 9:
        return math.inf
    variance_east = float(covariance[0])
    variance_north = float(covariance[4])
    if not (
        math.isfinite(variance_east)
        and math.isfinite(variance_north)
        and variance_east > 0.0
        and variance_north > 0.0
    ):
        return math.inf
    return max(math.sqrt(variance_east), math.sqrt(variance_north))


class WaypointController(Node):
    """Generate zero-throttle steering requests toward a GPS waypoint."""

    def __init__(self):
        super().__init__("waypoint_controller")

        self.declare_parameter("steering_kp", 0.8)
        self.declare_parameter("max_steering", 0.5)
        self.declare_parameter("heading_deadband_deg", 5.0)
        self.declare_parameter("arrival_radius_m", 2.0)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("data_timeout_s", 1.0)
        self.declare_parameter("output_hz", 5.0)
        # max_gps_sigma_m is the legacy name for the reliable threshold.
        # Declaring the new parameter from its resolved value preserves old
        # launch-file overrides while exposing the Schmitt-trigger interface.
        self.declare_parameter("max_gps_sigma_m", 6.0)
        legacy_reliable_sigma = float(
            self.get_parameter("max_gps_sigma_m").value
        )
        self.declare_parameter(
            "gps_sigma_reliable_m", legacy_reliable_sigma
        )
        self.declare_parameter("gps_sigma_unreliable_m", 8.0)
        self.declare_parameter("gps_sigma_hard_stop_m", 12.0)
        self.declare_parameter("gps_recovery_seconds", 0.5)
        self.declare_parameter("gps_failure_seconds", 0.5)
        self.declare_parameter("gps_degraded_grace_seconds", 2.5)
        self.declare_parameter("vision_timeout_seconds", 1.25)
        self.declare_parameter("safety_status_timeout_seconds", 0.5)
        self.declare_parameter("arrival_confirm_count", 5)
        self.declare_parameter("drive_enabled", False)
        self.declare_parameter("test_throttle", 0.05)
        self.declare_parameter("max_test_throttle", 0.10)
        self.declare_parameter("drive_heading_limit_deg", 20.0)
        self.declare_parameter("drive_heading_slow_limit_deg", 30.0)
        self.declare_parameter("turning_test_throttle", 0.03)
        self.declare_parameter("first_drive_max_seconds", 0.0)

        self.steering_kp = float(
            self.get_parameter("steering_kp").value
        )
        self.max_steering = float(
            self.get_parameter("max_steering").value
        )
        self.heading_deadband = math.radians(
            float(self.get_parameter("heading_deadband_deg").value)
        )
        self.arrival_radius = float(
            self.get_parameter("arrival_radius_m").value
        )
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.data_timeout = float(
            self.get_parameter("data_timeout_s").value
        )
        output_hz = float(self.get_parameter("output_hz").value)
        self.gps_sigma_reliable = float(
            self.get_parameter("gps_sigma_reliable_m").value
        )
        self.max_gps_sigma = self.gps_sigma_reliable
        self.gps_sigma_unreliable = float(
            self.get_parameter("gps_sigma_unreliable_m").value
        )
        self.gps_sigma_hard_stop = float(
            self.get_parameter("gps_sigma_hard_stop_m").value
        )
        self.gps_recovery_seconds = float(
            self.get_parameter("gps_recovery_seconds").value
        )
        self.gps_failure_seconds = float(
            self.get_parameter("gps_failure_seconds").value
        )
        self.gps_degraded_grace_seconds = float(
            self.get_parameter("gps_degraded_grace_seconds").value
        )
        self.vision_timeout = float(
            self.get_parameter("vision_timeout_seconds").value
        )
        self.safety_status_timeout = float(
            self.get_parameter("safety_status_timeout_seconds").value
        )
        self.arrival_confirm_count = int(
            self.get_parameter("arrival_confirm_count").value
        )
        self.drive_enabled = bool(self.get_parameter("drive_enabled").value)
        self.test_throttle = float(self.get_parameter("test_throttle").value)
        self.max_test_throttle = float(
            self.get_parameter("max_test_throttle").value
        )
        self.drive_heading_limit = math.radians(
            float(self.get_parameter("drive_heading_limit_deg").value)
        )
        self.drive_heading_slow_limit = math.radians(
            float(
                self.get_parameter("drive_heading_slow_limit_deg").value
            )
        )
        self.turning_test_throttle = float(
            self.get_parameter("turning_test_throttle").value
        )
        self.first_drive_max_seconds = float(
            self.get_parameter("first_drive_max_seconds").value
        )
        self.first_drive_started = None
        self.first_drive_latched = False

        if self.steering_kp < 0.0:
            raise ValueError("steering_kp must be nonnegative")
        if self.max_steering <= 0.0:
            raise ValueError("max_steering must be positive")
        if self.heading_deadband < 0.0:
            raise ValueError("heading_deadband_deg must be nonnegative")
        if self.arrival_radius <= 0.0:
            raise ValueError("arrival_radius_m must be positive")
        if self.data_timeout <= 0.0:
            raise ValueError("data_timeout_s must be positive")
        if output_hz <= 0.0:
            raise ValueError("output_hz must be positive")
        if not 0.0 < self.gps_sigma_reliable < self.gps_sigma_unreliable:
            raise ValueError(
                "gps_sigma_reliable_m must be positive and strictly less "
                "than gps_sigma_unreliable_m"
            )
        if self.gps_sigma_hard_stop < self.gps_sigma_unreliable:
            raise ValueError(
                "gps_sigma_hard_stop_m must be at least "
                "gps_sigma_unreliable_m"
            )
        if self.gps_recovery_seconds <= 0.0:
            raise ValueError("gps_recovery_seconds must be positive")
        if self.gps_failure_seconds <= 0.0:
            raise ValueError("gps_failure_seconds must be positive")
        if self.gps_degraded_grace_seconds <= 0.0:
            raise ValueError("gps_degraded_grace_seconds must be positive")
        if self.vision_timeout <= 0.0:
            raise ValueError("vision_timeout_seconds must be positive")
        if self.safety_status_timeout <= 0.0:
            raise ValueError("safety_status_timeout_seconds must be positive")
        if self.arrival_confirm_count <= 0:
            raise ValueError("arrival_confirm_count must be positive")
        if not 0.0 <= self.test_throttle <= self.max_test_throttle:
            raise ValueError("test_throttle must be within its hard maximum")
        if not 0.0 < self.max_test_throttle <= 0.10:
            raise ValueError("max_test_throttle must be in (0, 0.10]")
        if not 0.0 < self.drive_heading_limit <= math.pi:
            raise ValueError("drive_heading_limit_deg must be in (0, 180]")
        if not self.drive_heading_limit < self.drive_heading_slow_limit <= math.pi:
            raise ValueError(
                "drive_heading_slow_limit_deg must exceed "
                "drive_heading_limit_deg and be at most 180"
            )
        if not 0.0 <= self.turning_test_throttle <= self.test_throttle:
            raise ValueError(
                "turning_test_throttle must be between zero and test_throttle"
            )
        if self.first_drive_max_seconds < 0.0:
            raise ValueError("first_drive_max_seconds must be nonnegative")

        self.distance = None
        self.desired_bearing = None
        self.current_yaw = None
        self.raw_yaw = None
        self.current_east = None
        self.current_north = None
        self.distance_time = None
        self.bearing_time = None
        self.odom_time = None
        self.gps_sigma = math.inf
        self.gps_state = GPS_UNRELIABLE
        self.gps_unreliable = True
        self.gps_data_valid = False
        self.gps_time = None
        self.gps_recovery_started = None
        self.gps_failure_started = None
        self.gps_grace_started = None
        self.imu_time = None
        self.vision_heartbeat_time = None
        self.vision_hazard_time = None
        self.vision_hazard_state = None
        self.safety_status_time = None
        self.safety_status = None
        self.arrival_count = 0
        self.target_reached_latched = False
        self.target_key = None

        self.create_subscription(
            Float64,
            "/manta/waypoint/distance_m",
            self.distance_callback,
            10,
        )
        self.create_subscription(
            Float64,
            "/manta/waypoint/desired_bearing_rad",
            self.bearing_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/manta/fused_odom",
            self.odom_callback,
            20,
        )
        self.create_subscription(
            NavSatFix,
            "/fix",
            self.fix_callback,
            20,
        )
        self.create_subscription(Imu, "/imu", self.imu_callback, 20)
        self.create_subscription(
            Empty,
            "/manta/vision/heartbeat",
            self.vision_heartbeat_callback,
            10,
        )
        self.create_subscription(
            Int32,
            "/manta/vision/hazard_state",
            self.vision_hazard_callback,
            10,
        )
        self.create_subscription(
            String,
            "/manta/safety/status",
            self.safety_status_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            "/manta/waypoint/target_enu",
            self.target_callback,
            10,
        )

        self.command_pub = self.create_publisher(
            Twist,
            "/manta/nav/cmd_vel_raw",
            10,
        )
        self.timer = self.create_timer(1.0 / output_hz, self.control_loop)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(
            "Waypoint heading controller started | "
            f"dry_run={self.dry_run} | throttle locked at 0.0"
        )

    def parameter_callback(self, parameters):
        """Allow a deliberate runtime transition out of dry-run mode."""
        for parameter in parameters:
            if parameter.name == "dry_run":
                if not isinstance(parameter.value, bool):
                    return SetParametersResult(
                        successful=False,
                        reason="dry_run must be a boolean",
                    )
                self.dry_run = parameter.value
                self.get_logger().warning(
                    f"dry_run changed to {self.dry_run}; "
                    "throttle remains locked at 0.0"
                )
            elif parameter.name == "drive_enabled":
                if not isinstance(parameter.value, bool):
                    return SetParametersResult(
                        successful=False,
                        reason="drive_enabled must be a boolean",
                    )
                self.drive_enabled = parameter.value
                if not self.drive_enabled:
                    self.first_drive_started = None
                    self.first_drive_latched = False
            elif parameter.name == "test_throttle":
                value = float(parameter.value)
                if not 0.0 <= value <= self.max_test_throttle:
                    return SetParametersResult(
                        successful=False,
                        reason="test_throttle exceeds hard maximum",
                    )
                self.test_throttle = value
            elif parameter.name == "first_drive_max_seconds":
                value = float(parameter.value)
                if value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="first_drive_max_seconds must be nonnegative",
                    )
                self.first_drive_max_seconds = value
            elif parameter.name in (
                "max_gps_sigma_m", "gps_sigma_reliable_m"
            ):
                value = float(parameter.value)
                if not 0.0 < value < self.gps_sigma_unreliable:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "reliable GPS sigma must be positive and below "
                            "the unreliable threshold"
                        ),
                    )
                self.gps_sigma_reliable = value
                self.max_gps_sigma = value
        return SetParametersResult(successful=True)

    def distance_callback(self, msg):
        if math.isfinite(msg.data):
            self.distance = float(msg.data)
            self.distance_time = time.monotonic()
            if self.target_reached_latched:
                return
            if self.gps_unreliable:
                self.arrival_count = 0
                return
            if self.distance <= self.arrival_radius:
                self.arrival_count += 1
                if self.arrival_count >= self.arrival_confirm_count:
                    self.target_reached_latched = True
            else:
                self.arrival_count = 0

    def set_gps_state(self, state):
        """Update the explicit GPS state and compatibility boolean."""
        self.gps_state = state
        self.gps_unreliable = state == GPS_UNRELIABLE

    def set_gps_unreliable(self):
        """Fail GPS closed and clear any in-progress timing windows."""
        self.set_gps_state(GPS_UNRELIABLE)
        self.gps_recovery_started = None
        self.gps_failure_started = None
        self.gps_grace_started = None
        if not self.target_reached_latched:
            self.arrival_count = 0

    def update_gps_quality(self, sigma, now):
        """Advance the time-based Schmitt trigger with a valid GPS fix."""
        self.gps_sigma = float(sigma)

        if sigma >= self.gps_sigma_hard_stop:
            self.set_gps_unreliable()
            return

        if self.gps_state == GPS_UNRELIABLE:
            self.gps_failure_started = None
            if sigma <= self.gps_sigma_reliable:
                if self.gps_recovery_started is None:
                    self.gps_recovery_started = now
                elif (
                    now - self.gps_recovery_started
                    >= self.gps_recovery_seconds
                ):
                    self.set_gps_state(GPS_RELIABLE)
                    self.gps_recovery_started = None
            else:
                self.gps_recovery_started = None
            return

        if self.gps_state == GPS_DEGRADED_GRACE:
            if sigma <= self.gps_sigma_reliable:
                if self.gps_recovery_started is None:
                    self.gps_recovery_started = now
                elif (
                    now - self.gps_recovery_started
                    >= self.gps_recovery_seconds
                ):
                    self.set_gps_state(GPS_RELIABLE)
                    self.gps_recovery_started = None
                    self.gps_grace_started = None
            else:
                self.gps_recovery_started = None
            if (
                self.gps_state == GPS_DEGRADED_GRACE
                and now - self.gps_grace_started
                >= self.gps_degraded_grace_seconds
            ):
                self.set_gps_unreliable()
            return

        self.gps_recovery_started = None
        if sigma >= self.gps_sigma_unreliable:
            if self.gps_failure_started is None:
                self.gps_failure_started = now
            elif (
                now - self.gps_failure_started
                >= self.gps_failure_seconds
            ):
                self.set_gps_state(GPS_DEGRADED_GRACE)
                self.gps_grace_started = now
                self.gps_failure_started = None
        else:
            # The 6-8 m band, and values below it, retain RELIABLE state.
            self.gps_failure_started = None

    def fix_callback(self, msg):
        now = time.monotonic()
        self.gps_time = now
        sigma = horizontal_gps_sigma(msg)
        valid = (
            msg.status.status != NavSatStatus.STATUS_NO_FIX
            and math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and -90.0 <= msg.latitude <= 90.0
            and -180.0 <= msg.longitude <= 180.0
            and math.isfinite(sigma)
        )
        self.gps_data_valid = valid
        if not valid:
            self.gps_sigma = math.inf
            self.set_gps_unreliable()
            return
        self.update_gps_quality(sigma, now)

    def gps_is_reliable(self, now=None):
        """Return current GPS usability, immediately failing stale data."""
        if now is None:
            now = time.monotonic()
        fresh = (
            self.gps_time is not None
            and now - self.gps_time <= self.data_timeout
        )
        if not self.gps_data_valid or not fresh:
            self.set_gps_unreliable()
            return False
        if self.gps_state == GPS_DEGRADED_GRACE:
            if (
                now - self.gps_grace_started
                >= self.gps_degraded_grace_seconds
            ):
                self.set_gps_unreliable()
                return False
            return self.grace_conditions_met(now)
        return self.gps_state == GPS_RELIABLE

    def grace_conditions_met(self, now=None):
        """Require every navigation and downstream health input in grace."""
        if now is None:
            now = time.monotonic()
        localization_fresh = (
            self.odom_time is not None
            and now - self.odom_time <= self.data_timeout
        )
        imu_fresh = (
            self.imu_time is not None
            and now - self.imu_time <= self.data_timeout
        )
        target_fresh = (
            self.target_key is not None
            and self.distance_time is not None
            and self.bearing_time is not None
            and now - self.distance_time <= self.data_timeout
            and now - self.bearing_time <= self.data_timeout
        )
        vision_fresh = (
            self.vision_heartbeat_time is not None
            and self.vision_hazard_time is not None
            and now - self.vision_heartbeat_time <= self.vision_timeout
            and now - self.vision_hazard_time <= self.vision_timeout
            and self.vision_hazard_state in (
                VISION_CLEAR,
                VISION_VALID_DEPTH,
                VISION_DEPTH_UNKNOWN,
            )
        )
        safety_healthy = (
            self.safety_status_time is not None
            and now - self.safety_status_time <= self.safety_status_timeout
            and self.safety_status == "PASS"
        )
        return (
            localization_fresh
            and imu_fresh
            and target_fresh
            and vision_fresh
            and safety_healthy
        )

    def gps_grace_remaining(self, now=None):
        """Return remaining grace time, or zero outside grace."""
        if self.gps_state != GPS_DEGRADED_GRACE:
            return 0.0
        if now is None:
            now = time.monotonic()
        return max(
            0.0,
            self.gps_degraded_grace_seconds
            - (now - self.gps_grace_started),
        )

    def imu_callback(self, _msg):
        self.imu_time = time.monotonic()

    def vision_heartbeat_callback(self, _msg):
        self.vision_heartbeat_time = time.monotonic()

    def vision_hazard_callback(self, msg):
        self.vision_hazard_state = int(msg.data)
        self.vision_hazard_time = time.monotonic()

    def safety_status_callback(self, msg):
        self.safety_status = str(msg.data)
        self.safety_status_time = time.monotonic()

    def target_callback(self, msg):
        target_key = (float(msg.point.x), float(msg.point.y))
        if self.target_key is None:
            self.target_key = target_key
        if math.hypot(
            target_key[0] - self.target_key[0],
            target_key[1] - self.target_key[1],
        ) > 1e-3:
            self.target_key = target_key
            self.arrival_count = 0
            self.target_reached_latched = False
        self.update_target_geometry()

    def update_target_geometry(self):
        """Refresh target geometry from the retained ENU target and pose."""
        if (
            self.target_key is None
            or self.current_east is None
            or self.current_north is None
        ):
            return
        delta_east = self.target_key[0] - self.current_east
        delta_north = self.target_key[1] - self.current_north
        self.distance_callback(
            Float64(data=math.hypot(delta_east, delta_north))
        )
        self.bearing_callback(
            Float64(data=math.atan2(delta_north, delta_east))
        )

    def bearing_callback(self, msg):
        if math.isfinite(msg.data):
            self.desired_bearing = wrap_angle(float(msg.data))
            self.bearing_time = time.monotonic()

    def odom_callback(self, msg):
        yaw = quaternion_yaw(msg.pose.pose.orientation)
        if math.isfinite(yaw):
            self.raw_yaw = wrap_angle(yaw)
            self.current_yaw = self.raw_yaw
            self.current_east = float(msg.pose.pose.position.x)
            self.current_north = float(msg.pose.pose.position.y)
            self.odom_time = time.monotonic()
            self.update_target_geometry()

    def data_is_fresh(self):
        now = time.monotonic()
        timestamps = (
            self.distance_time,
            self.bearing_time,
            self.odom_time,
        )
        return all(
            stamp is not None and now - stamp <= self.data_timeout
            for stamp in timestamps
        )

    def calculate_control(self):
        """Return state, requested steering, and heading error."""
        requested_steering = 0.0
        heading_error = None

        if self.target_reached_latched:
            state = "TARGET_REACHED"
        elif not self.gps_is_reliable():
            state = "GPS_UNRELIABLE"
        elif not self.data_is_fresh():
            state = "WAITING_FOR_LOCALIZATION"
        else:
            heading_error = wrap_angle(
                self.desired_bearing - self.current_yaw
            )
            if abs(heading_error) <= self.heading_deadband:
                state = "ALIGNED"
            else:
                # Positive ENU error is left/CCW, while this car uses
                # positive angular.z for right steering.
                requested_steering = max(
                    -self.max_steering,
                    min(
                        self.max_steering,
                        -self.steering_kp * heading_error,
                    ),
                )
                state = (
                    "TURN_RIGHT"
                    if requested_steering > 0.0
                    else "TURN_LEFT"
                )

        return state, requested_steering, heading_error

    def make_command(self, requested_steering, state=None, heading_error=None):
        """Build a bounded command, with propulsion disabled by default."""
        command = Twist()
        can_drive = (
            self.drive_enabled
            and not self.dry_run
            and self.gps_is_reliable()
            and state not in (None, "GPS_UNRELIABLE", "WAITING_FOR_LOCALIZATION",
                              "TARGET_REACHED")
            and self.target_key is not None
            and self.distance is not None
            and self.distance > self.arrival_radius
            and heading_error is not None
            and abs(heading_error) <= self.drive_heading_slow_limit
        )
        if can_drive and self.first_drive_latched:
            can_drive = False
        if can_drive and self.first_drive_max_seconds > 0.0:
            now = time.monotonic()
            if self.first_drive_started is None:
                self.first_drive_started = now
            elif now - self.first_drive_started >= self.first_drive_max_seconds:
                self.first_drive_latched = True
                can_drive = False
        if can_drive and abs(heading_error) > self.drive_heading_limit:
            command.linear.x = self.turning_test_throttle
        else:
            command.linear.x = self.test_throttle if can_drive else 0.0
        if self.dry_run:
            command.angular.z = 0.0
        elif requested_steering < 0.0:
            # Logical LEFT: 0 -> physical straight (-0.5),
            # -0.5 -> calibrated full-left command (-1.3).
            command.angular.z = -0.5 + 1.6 * requested_steering
        else:
            # Logical RIGHT: 0 -> physical straight (-0.5),
            # +0.5 -> calibrated full-right command (+0.1).
            command.angular.z = -0.5 + 1.2 * requested_steering
        return command

    def control_loop(self):
        state, requested_steering, heading_error = (
            self.calculate_control()
        )

        command = self.make_command(requested_steering, state, heading_error)
        self.command_pub.publish(command)

        distance_text = (
            "unavailable"
            if self.distance is None
            else f"{self.distance:.2f} m"
        )
        desired_text = (
            "unavailable"
            if self.desired_bearing is None
            else f"{math.degrees(self.desired_bearing):.1f} deg"
        )
        current_text = (
            "unavailable"
            if self.current_yaw is None
            else f"{math.degrees(self.current_yaw):.1f} deg"
        )
        error_text = (
            "unavailable"
            if heading_error is None
            else f"{math.degrees(heading_error):.1f} deg"
        )
        gps_sigma_text = (
            "unavailable"
            if not math.isfinite(self.gps_sigma)
            else f"{self.gps_sigma:.2f} m"
        )
        command_text = (
            "STOP"
            if state in (
                "GPS_UNRELIABLE",
                "WAITING_FOR_LOCALIZATION",
                "TARGET_REACHED",
            )
            else "DRY RUN" if self.dry_run else "STEERING"
        )
        gps_quality_state = self.gps_state
        grace_remaining = self.gps_grace_remaining()
        print(
            "\n"
            f"Observation time: {time.time():.3f}\n"
            f"Distance: {distance_text}\n"
            f"GPS sigma: {gps_sigma_text}\n"
            f"GPS quality state: {gps_quality_state}\n"
            f"GPS grace remaining: {grace_remaining:.2f} s\n"
            f"Desired heading: {desired_text}\n"
            f"Current heading: {current_text}\n"
            f"Heading error: {error_text}\n"
            f"Requested steering: {requested_steering:.3f}\n"
            f"State: {state}\n"
            f"Command: {command_text}",
            flush=True,
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
