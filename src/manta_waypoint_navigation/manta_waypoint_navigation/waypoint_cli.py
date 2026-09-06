#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64


EARTH_RADIUS_M = 6378137.0


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


class WaypointCli(Node):
    """Report a GPS target relative to MANTA fused localization."""

    def __init__(self, target_lat, target_lon, arrival_radius):
        super().__init__("manta_waypoint_cli")

        self.target_lat = float(target_lat)
        self.target_lon = float(target_lon)
        self.arrival_radius = float(arrival_radius)

        self.current_fix = None
        self.current_odom = None
        self.fix_arrival_time = None
        self.odom_arrival_time = None
        self.origin_lat_rad = None
        self.origin_lon_rad = None
        self.target_east = None
        self.target_north = None
        self.target_reached = False

        self.create_subscription(
            NavSatFix,
            "/fix",
            self.fix_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/manta/fused_odom",
            self.odom_callback,
            20,
        )

        self.target_pub = self.create_publisher(
            PointStamped,
            "/manta/waypoint/target_enu",
            10,
        )
        self.distance_pub = self.create_publisher(
            Float64,
            "/manta/waypoint/distance_m",
            10,
        )
        self.desired_bearing_pub = self.create_publisher(
            Float64,
            "/manta/waypoint/desired_bearing_rad",
            10,
        )
        self.heading_error_pub = self.create_publisher(
            Float64,
            "/manta/waypoint/heading_error_rad",
            10,
        )
        self.reached_pub = self.create_publisher(
            Bool,
            "/manta/waypoint/reached",
            10,
        )

        self.timer = self.create_timer(0.5, self.report_solution)
        self.get_logger().info(
            "Waypoint reporting started; no motor command publisher exists."
        )

    def fix_callback(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if not (
            math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
        ):
            return

        self.current_fix = msg
        self.fix_arrival_time = time.monotonic()
        self.try_establish_origin()

    def odom_callback(self, msg):
        self.current_odom = msg
        self.odom_arrival_time = time.monotonic()
        self.try_establish_origin()

    def try_establish_origin(self):
        if self.origin_lat_rad is not None:
            return
        if self.current_fix is None or self.current_odom is None:
            return
        if abs(self.fix_arrival_time - self.odom_arrival_time) > 0.5:
            return

        current_lat = math.radians(self.current_fix.latitude)
        current_lon = math.radians(self.current_fix.longitude)
        current_east = self.current_odom.pose.pose.position.x
        current_north = self.current_odom.pose.pose.position.y

        self.origin_lat_rad = (
            current_lat - current_north / EARTH_RADIUS_M
        )
        cos_origin = math.cos(self.origin_lat_rad)
        if abs(cos_origin) <= 1e-9:
            self.origin_lat_rad = None
            return

        self.origin_lon_rad = (
            current_lon
            - current_east / (EARTH_RADIUS_M * cos_origin)
        )
        self.target_east, self.target_north = self.gps_to_enu(
            self.target_lat,
            self.target_lon,
        )

        self.get_logger().info(
            "Aligned waypoint ENU origin with current fused map frame."
        )

    def gps_to_enu(self, latitude, longitude):
        lat = math.radians(latitude)
        lon = math.radians(longitude)
        east = (
            EARTH_RADIUS_M
            * (lon - self.origin_lon_rad)
            * math.cos(self.origin_lat_rad)
        )
        north = EARTH_RADIUS_M * (lat - self.origin_lat_rad)
        return east, north

    def report_solution(self):
        if (
            self.current_fix is None
            or self.current_odom is None
            or self.target_east is None
        ):
            self.get_logger().info("Waiting for valid /fix and fused odometry...")
            return

        current_east = self.current_odom.pose.pose.position.x
        current_north = self.current_odom.pose.pose.position.y
        delta_east = self.target_east - current_east
        delta_north = self.target_north - current_north
        distance = math.hypot(delta_east, delta_north)
        desired_bearing = math.atan2(delta_north, delta_east)
        current_yaw = quaternion_yaw(
            self.current_odom.pose.pose.orientation
        )
        heading_error = wrap_angle(desired_bearing - current_yaw)
        reached = distance <= self.arrival_radius
        gps_sigma = horizontal_gps_sigma(self.current_fix)
        gps_sigma_text = (
            "unavailable"
            if not math.isfinite(gps_sigma)
            else f"{gps_sigma:.2f} m"
        )

        print(
            "\n"
            f"Current GPS: {self.current_fix.latitude:.9f}, "
            f"{self.current_fix.longitude:.9f}\n"
            f"Target GPS: {self.target_lat:.9f}, "
            f"{self.target_lon:.9f}\n"
            f"Current ENU: east={current_east:.2f} m, "
            f"north={current_north:.2f} m\n"
            f"Target ENU: east={self.target_east:.2f} m, "
            f"north={self.target_north:.2f} m\n"
            f"Distance to target: {distance:.2f} m\n"
            f"GPS sigma: {gps_sigma_text}\n"
            f"Current yaw: {math.degrees(current_yaw):.1f} deg\n"
            f"Desired bearing: {math.degrees(desired_bearing):.1f} deg\n"
            f"Heading error: {math.degrees(heading_error):.1f} deg",
            flush=True,
        )

        if reached:
            print("TARGET REACHED", flush=True)
        if reached and not self.target_reached:
            self.get_logger().info("TARGET REACHED")
        self.target_reached = reached

        target_msg = PointStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = "map"
        target_msg.point.x = self.target_east
        target_msg.point.y = self.target_north
        self.target_pub.publish(target_msg)

        distance_msg = Float64()
        distance_msg.data = distance
        self.distance_pub.publish(distance_msg)

        bearing_msg = Float64()
        bearing_msg.data = desired_bearing
        self.desired_bearing_pub.publish(bearing_msg)

        heading_msg = Float64()
        heading_msg.data = heading_error
        self.heading_error_pub.publish(heading_msg)

        reached_msg = Bool()
        reached_msg.data = reached
        self.reached_pub.publish(reached_msg)


def parse_arguments(arguments):
    """Parse CLI arguments while preserving ROS-specific arguments."""
    parser = argparse.ArgumentParser(
        description="Report a GPS Point-B navigation solution."
    )
    parser.add_argument("--lat", type=float, required=True)
    parser.add_argument("--lon", type=float, required=True)
    parser.add_argument(
        "--arrival-radius",
        type=float,
        default=2.0,
        help="Target arrival radius in meters (default: 2.0).",
    )
    parsed, ros_arguments = parser.parse_known_args(arguments)

    if not -90.0 <= parsed.lat <= 90.0:
        parser.error("--lat must be between -90 and 90 degrees")
    if not -180.0 <= parsed.lon <= 180.0:
        parser.error("--lon must be between -180 and 180 degrees")
    if parsed.arrival_radius <= 0.0:
        parser.error("--arrival-radius must be positive")
    return parsed, ros_arguments


def main(args=None):
    cli_args = sys.argv[1:] if args is None else args
    parsed, ros_arguments = parse_arguments(cli_args)
    rclpy.init(args=ros_arguments)
    node = WaypointCli(
        parsed.lat,
        parsed.lon,
        parsed.arrival_radius,
    )
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
