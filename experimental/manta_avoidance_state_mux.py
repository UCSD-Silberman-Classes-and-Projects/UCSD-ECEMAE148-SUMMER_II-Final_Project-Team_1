#!/usr/bin/env python3

import time
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32


class Mux(Node):

    def __init__(self):
        super().__init__('manta_avoidance_state_mux')

        self.count = 0
        self.count_t = 0.0

        self.hazard = 0
        self.hazard_t = 0.0

        self.lane = False
        self.lane_t = 0.0

        self.cmd = Twist()
        self.cmd_t = 0.0

        self.last_two_cone = 0.0
        self.active = False
        self.last_log = None

        self.create_subscription(
            Int32, '/cone/count_0',
            self.count_cb, 10)

        self.create_subscription(
            Int32, '/manta/vision/hazard_state',
            self.hazard_cb, 10)

        self.create_subscription(
            Bool, '/manta/avoidance/lane_active',
            self.lane_cb, 10)

        self.create_subscription(
            Twist, '/manta/nav/cmd_vel_avoidance',
            self.cmd_cb, 10)

        self.pub = self.create_publisher(
            Bool, '/manta/avoidance/active', 10)

        self.create_timer(0.05, self.tick)

        self.get_logger().info(
            'MANTA avoidance mux started with hysteresis'
        )

    def count_cb(self, m):
        self.count = int(m.data)
        self.count_t = time.monotonic()

    def hazard_cb(self, m):
        self.hazard = int(m.data)
        self.hazard_t = time.monotonic()

    def lane_cb(self, m):
        self.lane = bool(m.data)
        self.lane_t = time.monotonic()

    def cmd_cb(self, m):
        self.cmd = m
        self.cmd_t = time.monotonic()

    def tick(self):
        now = time.monotonic()

        count_two = (
            now - self.count_t < 1.5
            and self.count >= 2
        )

        hazard_two = (
            now - self.hazard_t < 1.5
            and self.hazard == 2
        )

        two_cones = count_two or hazard_two

        if two_cones:
            self.last_two_cone = now

        lane_active = (
            self.lane
            and now - self.lane_t < 1.0
        )

        cmd_fresh = (
            now - self.cmd_t < 1.5
            and abs(float(self.cmd.angular.z)) >= 0.05
        )

        # Initial entry requires real two-cone evidence
        # AND a real avoidance steering command.
        if (two_cones and cmd_fresh) or lane_active:
            self.active = True

        # Once entered, hold maneuver mode through brief
        # perception/controller gaps.
        elif self.active:
            if now - self.last_two_cone > 2.0:
                self.active = False

        self.pub.publish(Bool(data=self.active))

        state = (
            self.active,
            self.count,
            self.hazard,
            round(float(self.cmd.angular.z), 2)
        )

        if state != self.last_log:
            self.get_logger().info(
                f'ACTIVE={self.active} '
                f'count={self.count} '
                f'hazard={self.hazard} '
                f'z={self.cmd.angular.z:+.2f}'
            )
            self.last_log = state


def main():
    rclpy.init()
    n = Mux()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
