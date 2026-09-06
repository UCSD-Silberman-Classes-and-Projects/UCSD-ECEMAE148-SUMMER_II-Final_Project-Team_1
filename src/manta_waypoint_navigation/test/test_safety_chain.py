import time
import unittest

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Int32

from manta_waypoint_navigation.waypoint_controller import WaypointController
from ucsd_robocar_control2_pkg.manta_safety_gate import MantaSafetyGate


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class SafetyChainTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.controller = WaypointController()
        self.gate = MantaSafetyGate()
        self.gate.cmd_pub = CapturePublisher()
        self.gate.latched_pub = CapturePublisher()
        self.gate.status_pub = CapturePublisher()

    def tearDown(self):
        self.gate.destroy_node()
        self.controller.destroy_node()

    def output(self):
        self.gate.control_loop()
        return self.gate.cmd_pub.messages[-1]

    def make_healthy(self):
        self.gate.heartbeat_callback(Empty())
        self.gate.hazard_state_callback(Int32(data=0))

    def test_startup_latch_zeros_throttle_and_steering(self):
        raw = Twist()
        raw.linear.x = 0.4
        raw.angular.z = 0.3
        self.gate.raw_cmd_callback(raw)
        output = self.output()
        self.assertEqual(output.linear.x, 0.0)
        self.assertEqual(output.angular.z, 0.0)

    def test_stale_safety_input_zeros_both_channels(self):
        self.make_healthy()
        self.gate.reset_callback(Empty())
        raw = Twist()
        raw.angular.z = 0.3
        self.gate.raw_cmd_callback(raw)
        self.gate.last_heartbeat_time = time.monotonic() - 10.0
        output = self.output()
        self.assertEqual(output.linear.x, 0.0)
        self.assertEqual(output.angular.z, 0.0)

    def test_healthy_chain_passes_only_steering(self):
        self.controller.dry_run = False
        controller_command = self.controller.make_command(0.3)
        self.assertEqual(controller_command.linear.x, 0.0)
        self.make_healthy()
        self.gate.reset_callback(Empty())
        self.gate.raw_cmd_callback(controller_command)
        output = self.output()
        self.assertEqual(output.linear.x, 0.0)
        self.assertEqual(output.angular.z, 0.3)

    def test_healthy_chain_with_dry_run_stays_zero(self):
        controller_command = self.controller.make_command(0.3)
        self.make_healthy()
        self.gate.reset_callback(Empty())
        self.gate.raw_cmd_callback(controller_command)
        output = self.output()
        self.assertEqual(output.linear.x, 0.0)
        self.assertEqual(output.angular.z, 0.0)


if __name__ == "__main__":
    unittest.main()
