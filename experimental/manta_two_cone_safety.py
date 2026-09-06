#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class TwoConeSafety(Node):

    def __init__(self):
        super().__init__('manta_two_cone_safety')

        self.count = 0
        self.count_t = 0.0

        self.hazard = 0
        self.hazard_t = 0.0

        self.last_confirmed = 0.0
        self.last_state = None

        self.create_subscription(
            Int32,
            '/cone/count_0',
            self.count_cb,
            10
        )

        self.create_subscription(
            Int32,
            '/manta/vision/hazard_state',
            self.hazard_cb,
            10
        )

        self.pub = self.create_publisher(
            Bool,
            '/manta/avoidance/safety_active',
            10
        )

        self.create_timer(0.05, self.tick)

        self.get_logger().info(
            'MANTA two-cone safety envelope started'
        )

    def count_cb(self, msg):
        self.count = int(msg.data)
        self.count_t = time.monotonic()

    def hazard_cb(self, msg):
        self.hazard = int(msg.data)
        self.hazard_t = time.monotonic()

    def tick(self):
        now = time.monotonic()

        count_ok = (
            self.count >= 2
            and now - self.count_t < 1.0
        )

        hazard_ok = (
            self.hazard == 2
            and now - self.hazard_t < 1.0
        )

        if count_ok and hazard_ok:
            self.last_confirmed = now

        # Hold through brief detector flicker.
        active = (
            self.last_confirmed > 0.0
            and now - self.last_confirmed < 2.0
        )

        self.pub.publish(Bool(data=active))

        if active != self.last_state:
            self.get_logger().info(
                f'SAFETY_ACTIVE={active} '
                f'count={self.count} '
                f'hazard={self.hazard}'
            )
            self.last_state = active


def main():
    rclpy.init()
    n = TwoConeSafety()

    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
