#!/usr/bin/env python3

import time
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Float32MultiArray


CONF_MIN = 0.25

THROTTLE = 0.15

KP = 1.2
MAX_STEER = 0.25
CENTER_TOL = 0.06

ALIGN_MAX_TIME = 1.0
CLEAR_TIME = 1.2
COOLDOWN_TIME = 2.0

PAIR_TIMEOUT = 0.50
COUNT_TIMEOUT = 0.75
WP_TIMEOUT = 1.0


class TwoConeMux(Node):

    IDLE = 0
    ALIGN = 1
    CLEAR = 2
    COOLDOWN = 3

    def __init__(self):
        super().__init__('manta_two_cone_nav_mux')

        self.state = self.IDLE
        self.state_t = time.monotonic()

        self.wp = Twist()
        self.wp_t = 0.0

        self.count = 0
        self.count_t = 0.0

        self.target_x = 0.5
        self.pair_t = 0.0

        self.last_mode = None

        self.create_subscription(
            Twist,
            '/manta/nav/cmd_vel_waypoint',
            self.wp_cb,
            10
        )

        self.create_subscription(
            Int32,
            '/cone/count_0',
            self.count_cb,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/cone/detections_0',
            self.det_cb,
            10
        )

        self.raw_pub = self.create_publisher(
            Twist,
            '/manta/nav/cmd_vel_raw',
            10
        )

        self.avoid_pub = self.create_publisher(
            Twist,
            '/manta/nav/cmd_vel_avoidance',
            10
        )

        self.safety_pub = self.create_publisher(
            Bool,
            '/manta/avoidance/safety_active',
            10
        )

        self.create_timer(0.05, self.tick)

        self.get_logger().info(
            'ONE-SHOT TWO-CONE CONTROLLER STARTED'
        )

    def wp_cb(self, msg):
        self.wp = msg
        self.wp_t = time.monotonic()

    def count_cb(self, msg):
        self.count = int(msg.data)
        self.count_t = time.monotonic()

    def det_cb(self, msg):
        d = list(msg.data)

        if len(d) < 10 or len(d) % 5 != 0:
            return

        boxes = []

        for i in range(0, len(d), 5):
            xmin = float(d[i])
            ymin = float(d[i+1])
            xmax = float(d[i+2])
            ymax = float(d[i+3])
            conf = float(d[i+4])

            if conf < CONF_MIN:
                continue

            if not (
                0.0 <= xmin < xmax <= 1.0
                and 0.0 <= ymin < ymax <= 1.0
            ):
                continue

            cx = 0.5 * (xmin + xmax)

            boxes.append(
                (conf, cx, xmin, xmax)
            )

        if len(boxes) < 2:
            return

        boxes.sort(
            key=lambda b: b[0],
            reverse=True
        )

        pair = boxes[:2]
        pair.sort(key=lambda b: b[1])

        left = pair[0]
        right = pair[1]

        if right[1] - left[1] < 0.05:
            return

        # Aim at gap between inner cone edges.
        gap_x = 0.5 * (
            left[3] + right[2]
        )

        self.target_x = gap_x
        self.pair_t = time.monotonic()

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def cmd(self, throttle, steering):
        m = Twist()
        m.linear.x = float(throttle)
        m.angular.z = float(steering)

        self.avoid_pub.publish(m)
        self.raw_pub.publish(m)

    def set_state(self, state, name):
        self.state = state
        self.state_t = time.monotonic()

        self.get_logger().info(
            f'MODE={name}'
        )

    def tick(self):
        now = time.monotonic()

        pair_fresh = (
            self.pair_t > 0.0
            and now - self.pair_t < PAIR_TIMEOUT
        )

        count_two = (
            self.count >= 2
            and now - self.count_t < COUNT_TIMEOUT
        )

        wp_fresh = (
            now - self.wp_t < WP_TIMEOUT
        )

        # ========================================
        # IDLE / GPS
        # ========================================

        if self.state == self.IDLE:

            self.safety_pub.publish(
                Bool(data=False)
            )

            if count_two and pair_fresh:
                self.set_state(
                    self.ALIGN,
                    'ALIGN'
                )
                return

            if wp_fresh:
                self.raw_pub.publish(self.wp)
            else:
                self.raw_pub.publish(Twist())

            self.avoid_pub.publish(Twist())
            return

        # ========================================
        # ALIGN TO GAP
        # ========================================

        if self.state == self.ALIGN:

            self.safety_pub.publish(
                Bool(data=True)
            )

            elapsed = now - self.state_t

            if pair_fresh:
                error = self.target_x - 0.5
            else:
                error = 0.0

            # If aligned OR alignment has lasted long enough:
            # STOP TURNING and commit straight through.
            if (
                abs(error) <= CENTER_TOL
                or elapsed >= ALIGN_MAX_TIME
            ):
                self.set_state(
                    self.CLEAR,
                    'CLEAR_STRAIGHT'
                )

                self.cmd(
                    THROTTLE,
                    0.0
                )
                return

            steer = self.clamp(
                KP * error,
                -MAX_STEER,
                MAX_STEER
            )

            self.cmd(
                THROTTLE,
                steer
            )

            return

        # ========================================
        # CLEARANCE
        #
        # CRITICAL:
        # ABSOLUTELY NO STEERING HERE.
        # Ignore cone geometry completely.
        # ========================================

        if self.state == self.CLEAR:

            self.safety_pub.publish(
                Bool(data=True)
            )

            self.cmd(
                THROTTLE,
                0.0
            )

            if now - self.state_t >= CLEAR_TIME:
                self.set_state(
                    self.COOLDOWN,
                    'COOLDOWN'
                )

            return

        # ========================================
        # COOLDOWN
        #
        # Ignore the same cones so they cannot
        # immediately retrigger another turn.
        # GPS resumes here.
        # ========================================

        if self.state == self.COOLDOWN:

            self.safety_pub.publish(
                Bool(data=False)
            )

            self.avoid_pub.publish(
                Twist()
            )

            if wp_fresh:
                self.raw_pub.publish(
                    self.wp
                )
            else:
                self.raw_pub.publish(
                    Twist()
                )

            if now - self.state_t >= COOLDOWN_TIME:
                self.set_state(
                    self.IDLE,
                    'WAYPOINT'
                )

            return


def main():
    rclpy.init()
    n = TwoConeMux()

    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
