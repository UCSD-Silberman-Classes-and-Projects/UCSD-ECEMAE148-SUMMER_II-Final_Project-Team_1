#!/usr/bin/env python3

import time
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray

SPEED = 0.08

KP = 1.25
MAX_STEER = 0.22
DEADBAND = 0.025

DETECTION_TIMEOUT = 0.35

# Only used AFTER we were centered and then lose the cones.
CENTER_LOCK = 0.055
LOSS_PASS_TIME = 0.60


class GateDemo(Node):

    def __init__(self):
        super().__init__('manta_gate_demo')

        self.target = None
        self.det_time = 0.0

        self.last_error = 0.0
        self.was_centered = False
        self.loss_start = None

        self.raw_pub = self.create_publisher(
            Twist,
            '/manta/nav/cmd_vel_raw',
            10
        )

        self.active_pub = self.create_publisher(
            Bool,
            '/manta/avoidance/safety_active',
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/cone/detections_0',
            self.det_cb,
            10
        )

        self.create_timer(0.05, self.tick)

        self.last_log = 0.0

        self.get_logger().info(
            'FINAL CLOSED-LOOP GATE DEMO READY'
        )

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

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

            if conf < 0.25:
                continue

            if not (
                0.0 <= xmin < xmax <= 1.0
                and 0.0 <= ymin < ymax <= 1.0
            ):
                continue

            cx = 0.5 * (xmin + xmax)

            boxes.append(
                (conf, cx)
            )

        if len(boxes) < 2:
            return

        # Use the two strongest cone detections.
        boxes.sort(
            key=lambda b: b[0],
            reverse=True
        )

        pair = boxes[:2]
        pair.sort(key=lambda b: b[1])

        left = pair[0][1]
        right = pair[1][1]

        if right - left < 0.04:
            return

        # Continuous target = midpoint of cone centers.
        self.target = 0.5 * (left + right)
        self.det_time = time.monotonic()
        self.loss_start = None

    def publish(self, x, z):

        cmd = Twist()

        cmd.linear.x = float(x)
        cmd.angular.z = float(z)

        self.raw_pub.publish(cmd)

    def tick(self):

        now = time.monotonic()

        fresh = (
            self.target is not None
            and
            now - self.det_time <= DETECTION_TIMEOUT
        )

        # ==========================================
        # TWO CONES VISIBLE:
        # CONTINUOUS CLOSED-LOOP STEERING
        # ==========================================

        if fresh:

            self.active_pub.publish(
                Bool(data=True)
            )

            error = self.target - 0.5

            if abs(error) < DEADBAND:
                steer = 0.0
            else:
                # Known vehicle convention:
                # negative = left
                # positive = right
                steer = KP * error

            steer = self.clamp(
                steer,
                -MAX_STEER,
                MAX_STEER
            )

            self.last_error = error

            if abs(error) <= CENTER_LOCK:
                self.was_centered = True

            self.publish(
                SPEED,
                steer
            )

            if now - self.last_log > 0.25:

                self.get_logger().info(
                    f'TRACK '
                    f'target={self.target:.3f} '
                    f'error={error:+.3f} '
                    f'steer={steer:+.3f}'
                )

                self.last_log = now

            return

        # ==========================================
        # CONES DISAPPEARED ONLY AFTER WE CENTERED:
        # short straight pass-through.
        # NO RETAINED TURN.
        # ==========================================

        if self.was_centered:

            if self.loss_start is None:
                self.loss_start = now

            if now - self.loss_start < LOSS_PASS_TIME:

                self.active_pub.publish(
                    Bool(data=True)
                )

                self.publish(
                    SPEED,
                    0.0
                )

                return

        # ==========================================
        # OTHERWISE STOP.
        # ==========================================

        self.was_centered = False
        self.loss_start = None

        self.active_pub.publish(
            Bool(data=False)
        )

        self.publish(
            0.0,
            0.0
        )


def main():

    rclpy.init()

    node = GateDemo()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Explicit zero before exit.
        for _ in range(5):
            node.publish(0.0, 0.0)
            time.sleep(0.02)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
