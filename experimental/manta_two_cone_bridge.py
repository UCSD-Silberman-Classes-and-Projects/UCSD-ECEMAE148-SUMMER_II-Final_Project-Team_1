#!/usr/bin/env python3

import time
import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class TwoConeBridge(Node):

    def __init__(self):
        super().__init__("manta_two_cone_bridge")

        self.hazard_state = None
        self.hazard_time = None

        self.lane_active = False
        self.lane_time = None

        self.pub = self.create_publisher(
            Bool,
            "/manta/avoidance/safety_active",
            10
        )

        self.create_subscription(
            Int32,
            "/manta/vision/hazard_state",
            self.hazard_cb,
            10
        )

        self.create_subscription(
            Bool,
            "/manta/avoidance/active",
            self.lane_cb,
            10
        )

        self.create_timer(0.05, self.tick)

        self.last_output = None

        self.get_logger().info(
            "Two-cone maneuver bridge started"
        )

    def hazard_cb(self, msg):
        self.hazard_state = int(msg.data)
        self.hazard_time = time.monotonic()

    def lane_cb(self, msg):
        self.lane_active = bool(msg.data)
        self.lane_time = time.monotonic()

    def tick(self):
        now = time.monotonic()

        two_cones = (
            self.hazard_state == 2
            and self.hazard_time is not None
            and now - self.hazard_time <= 1.0
        )

        lane = (
            self.lane_active
            and self.lane_time is not None
            and now - self.lane_time <= 1.0
        )

        active = bool(two_cones or lane)

        msg = Bool()
        msg.data = active
        self.pub.publish(msg)

        if active != self.last_output:
            self.get_logger().info(
                f"SAFETY_MANEUVER={active} "
                f"hazard={self.hazard_state} "
                f"lane={self.lane_active}"
            )
            self.last_output = active


def main():
    rclpy.init()
    node = TwoConeBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
