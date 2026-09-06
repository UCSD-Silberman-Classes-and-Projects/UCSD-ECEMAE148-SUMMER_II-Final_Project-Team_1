#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Arbiter(Node):

    def __init__(self):
        super().__init__('manta_cmd_arbiter')

        self.wp = Twist()
        self.avoid = Twist()

        self.wp_t = 0.0
        self.avoid_t = 0.0

        self.mode = None

        self.create_subscription(
            Twist,
            '/manta/nav/cmd_vel_waypoint',
            self.wp_cb,
            10
        )

        self.create_subscription(
            Twist,
            '/manta/nav/cmd_vel_avoidance',
            self.avoid_cb,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/manta/nav/cmd_vel_raw',
            10
        )

        self.create_timer(0.05, self.tick)

        self.get_logger().info(
            'MANTA arbiter started: GPS until real avoidance steering'
        )

    def wp_cb(self, msg):
        self.wp = msg
        self.wp_t = time.monotonic()

    def avoid_cb(self, msg):
        self.avoid = msg
        self.avoid_t = time.monotonic()

    def tick(self):
        now = time.monotonic()

        wp_fresh = now - self.wp_t < 1.0
        avoid_fresh = now - self.avoid_t < 1.0

        # Lane controller previously produced about -0.10 to -0.30
        # when it actually wanted to maneuver.
        avoid_steering = (
            avoid_fresh
            and abs(float(self.avoid.angular.z)) >= 0.05
        )

        if avoid_steering:
            self.pub.publish(self.avoid)
            mode = 'AVOIDANCE'

        elif wp_fresh:
            self.pub.publish(self.wp)
            mode = 'WAYPOINT'

        else:
            self.pub.publish(Twist())
            mode = 'ZERO'

        if mode != self.mode:
            self.get_logger().info(
                f'MODE={mode} '
                f'avoid_z={self.avoid.angular.z:+.2f}'
            )
            self.mode = mode


def main():
    rclpy.init()
    n = Arbiter()

    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
