#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from manta_waypoint_navigation.waypoint_controller import horizontal_gps_sigma


class GpsSigmaMonitor(Node):
    """Print conservative horizontal GNSS sigma from NavSatFix covariance."""

    def __init__(self):
        super().__init__("gps_sigma_monitor")
        self.declare_parameter("max_gps_sigma_m", 5.0)
        self.max_sigma = float(
            self.get_parameter("max_gps_sigma_m").value
        )
        self.create_subscription(NavSatFix, "/fix", self.fix_callback, 20)

    def fix_callback(self, msg):
        sigma = horizontal_gps_sigma(msg)
        if math.isfinite(sigma):
            state = "PASS" if sigma <= self.max_sigma else "WAIT"
            print(
                f"GPS sigma: {sigma:.2f} m | "
                f"limit: {self.max_sigma:.2f} m | {state}",
                flush=True,
            )
        else:
            print("GPS sigma: unavailable | WAIT", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = GpsSigmaMonitor()
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
