#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


RAW_TOPIC = "/manta/lidar/healthy"
STABLE_TOPIC = "/manta/lidar/healthy_stable"

# Ignore short false flickers.
# If we go more than 3 seconds without seeing a TRUE health sample,
# stabilized health becomes FALSE.
HOLD_SEC = 3.0


class MantaLidarHealthStabilizer(Node):

    def __init__(self):
        super().__init__("manta_lidar_health_stabilizer")

        self.last_true = None
        self.last_output = None

        self.create_subscription(
            Bool,
            RAW_TOPIC,
            self.health_cb,
            10,
        )

        self.pub = self.create_publisher(
            Bool,
            STABLE_TOPIC,
            10,
        )

        self.create_timer(0.05, self.tick)

        self.get_logger().info(
            "MANTA LiDAR health stabilizer started | "
            "raw false debounce = 3.0 s"
        )

    def health_cb(self, msg):
        if bool(msg.data):
            self.last_true = time.monotonic()

    def tick(self):

        stable = (
            self.last_true is not None
            and time.monotonic() - self.last_true <= HOLD_SEC
        )

        msg = Bool()
        msg.data = stable
        self.pub.publish(msg)

        if stable != self.last_output:
            self.get_logger().info(
                f"STABLE_HEALTH={stable}"
            )
            self.last_output = stable


def main():
    rclpy.init()
    node = MantaLidarHealthStabilizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
