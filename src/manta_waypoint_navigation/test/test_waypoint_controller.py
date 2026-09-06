import math
import time
import unittest

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from manta_waypoint_navigation.waypoint_controller import (
    GPS_DEGRADED_GRACE,
    GPS_RELIABLE,
    GPS_UNRELIABLE,
    WaypointController,
)


class WaypointControllerTest(unittest.TestCase):

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
        self.controller.gps_unreliable = False
        self.controller.gps_state = GPS_RELIABLE
        now = time.monotonic()
        self.controller.gps_data_valid = True
        self.controller.gps_time = now
        self.controller.distance = 10.0
        self.controller.desired_bearing = 0.0
        self.controller.current_yaw = 0.0
        self.controller.distance_time = now
        self.controller.bearing_time = now
        self.controller.odom_time = now
        self.controller.target_key = (0.0, 0.0)
        self.controller.imu_time = now
        self.controller.vision_heartbeat_time = now
        self.controller.vision_hazard_time = now
        self.controller.vision_hazard_state = 0
        self.controller.safety_status_time = now
        self.controller.safety_status = "PASS"

    def tearDown(self):
        self.controller.destroy_node()

    def test_left_target_requests_left(self):
        self.controller.desired_bearing = math.radians(30.0)
        state, steering, _ = self.controller.calculate_control()
        self.assertEqual(state, "TURN_LEFT")
        self.assertLess(steering, 0.0)

    def test_right_target_requests_right(self):
        self.controller.desired_bearing = math.radians(-30.0)
        state, steering, _ = self.controller.calculate_control()
        self.assertEqual(state, "TURN_RIGHT")
        self.assertGreater(steering, 0.0)

    def test_aligned_requests_zero(self):
        self.controller.desired_bearing = math.radians(2.0)
        state, steering, _ = self.controller.calculate_control()
        self.assertEqual(state, "ALIGNED")
        self.assertEqual(steering, 0.0)

    def test_unreliable_gps_stops(self):
        self.controller.set_gps_unreliable()
        state, steering, _ = self.controller.calculate_control()
        self.assertEqual(state, "GPS_UNRELIABLE")
        self.assertEqual(steering, 0.0)

    def make_fix(self, sigma, status=NavSatStatus.STATUS_FIX):
        fix = NavSatFix()
        fix.status.status = status
        fix.latitude = 32.0
        fix.longitude = -117.0
        fix.position_covariance[0] = sigma * sigma
        fix.position_covariance[4] = sigma * sigma
        return fix

    def quality_sample(self, sigma, now):
        self.controller.gps_data_valid = True
        self.controller.gps_time = now
        self.controller.update_gps_quality(sigma, now)

    def test_gps_starts_unreliable(self):
        controller = WaypointController()
        try:
            self.assertTrue(controller.gps_unreliable)
            self.assertEqual(controller.gps_state, GPS_UNRELIABLE)
        finally:
            controller.destroy_node()

    def test_sustained_good_gps_recovers_but_one_sample_does_not(self):
        self.controller.gps_unreliable = True
        self.controller.gps_state = GPS_UNRELIABLE
        self.quality_sample(5.8, 10.0)
        self.assertTrue(self.controller.gps_unreliable)
        self.quality_sample(5.9, 10.49)
        self.assertTrue(self.controller.gps_unreliable)
        self.quality_sample(5.7, 10.50)
        self.assertFalse(self.controller.gps_unreliable)
        self.assertEqual(self.controller.gps_state, GPS_RELIABLE)

    def test_hysteresis_band_stays_reliable(self):
        for index, sigma in enumerate((6.2, 6.8, 7.5, 6.4, 7.9)):
            self.quality_sample(sigma, 20.0 + index * 0.2)
        self.assertFalse(self.controller.gps_unreliable)

    def test_brief_high_spike_does_not_fail(self):
        self.quality_sample(8.2, 30.0)
        self.assertFalse(self.controller.gps_unreliable)
        self.quality_sample(7.9, 30.2)
        self.assertFalse(self.controller.gps_unreliable)

    def test_sustained_high_gps_fails(self):
        self.quality_sample(8.3, 40.0)
        self.quality_sample(8.4, 40.49)
        self.assertFalse(self.controller.gps_unreliable)
        self.quality_sample(8.2, 40.50)
        self.assertEqual(self.controller.gps_state, GPS_DEGRADED_GRACE)

    def test_hard_stop_is_immediate(self):
        self.quality_sample(12.0, 50.0)
        self.assertTrue(self.controller.gps_unreliable)

    def test_invalid_fix_is_immediate(self):
        self.controller.fix_callback(
            self.make_fix(1.0, NavSatStatus.STATUS_NO_FIX)
        )
        self.assertTrue(self.controller.gps_unreliable)
        self.assertFalse(self.controller.gps_data_valid)

    def test_stale_gps_immediately_zeros_throttle(self):
        self.controller.dry_run = False
        self.controller.drive_enabled = True
        self.controller.gps_time = time.monotonic() - 10.0
        command = self.controller.make_command(0.0, "ALIGNED", 0.0)
        self.assertEqual(command.linear.x, 0.0)
        self.assertTrue(self.controller.gps_unreliable)

    def test_recovery_after_poor_gps(self):
        self.quality_sample(8.5, 60.0)
        self.quality_sample(8.5, 60.5)
        self.assertEqual(self.controller.gps_state, GPS_DEGRADED_GRACE)
        self.quality_sample(5.9, 61.0)
        self.quality_sample(5.8, 61.5)
        self.assertFalse(self.controller.gps_unreliable)

    def enter_grace(self, now):
        self.controller.set_gps_state(GPS_DEGRADED_GRACE)
        self.controller.gps_grace_started = now
        self.controller.gps_time = now
        self.controller.gps_data_valid = True
        self.controller.gps_sigma = 9.0
        self.controller.distance_time = now
        self.controller.bearing_time = now
        self.controller.odom_time = now
        self.controller.imu_time = now
        self.controller.vision_heartbeat_time = now
        self.controller.vision_hazard_time = now
        self.controller.safety_status_time = now

    def test_fresh_degraded_grace_allows_bounded_throttle(self):
        now = time.monotonic()
        self.enter_grace(now)
        self.controller.drive_enabled = True
        self.controller.dry_run = False
        command = self.controller.make_command(0.0, "ALIGNED", 0.0)
        self.assertEqual(command.linear.x, 0.05)

    def test_expired_degraded_grace_stops(self):
        now = time.monotonic()
        self.enter_grace(
            now - self.controller.gps_degraded_grace_seconds - 0.01
        )
        self.controller.gps_time = now
        self.controller.drive_enabled = True
        self.controller.dry_run = False
        command = self.controller.make_command(0.0, "ALIGNED", 0.0)
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(self.controller.gps_state, GPS_UNRELIABLE)

    def test_stale_imu_stops_degraded_grace(self):
        now = time.monotonic()
        self.enter_grace(now)
        self.controller.imu_time = now - 10.0
        self.controller.drive_enabled = True
        self.controller.dry_run = False
        command = self.controller.make_command(0.0, "ALIGNED", 0.0)
        self.assertEqual(command.linear.x, 0.0)

    def test_unhealthy_vision_stops_degraded_grace(self):
        now = time.monotonic()
        self.enter_grace(now)
        self.controller.vision_hazard_state = 99
        self.controller.drive_enabled = True
        self.controller.dry_run = False
        command = self.controller.make_command(0.0, "ALIGNED", 0.0)
        self.assertEqual(command.linear.x, 0.0)

    def test_target_enu_does_not_wander_with_raw_gps(self):
        target = PointStamped()
        target.point.x = 20.0
        target.point.y = -3.0
        self.controller.target_callback(target)
        fixed_target = self.controller.target_key
        self.controller.fix_callback(self.make_fix(7.0))
        noisy_fix = self.make_fix(7.5)
        noisy_fix.latitude += 0.001
        noisy_fix.longitude -= 0.001
        self.controller.fix_callback(noisy_fix)
        self.assertEqual(self.controller.target_key, fixed_target)

    def test_retained_target_geometry_refreshes_from_odometry(self):
        target = PointStamped()
        target.point.x = 30.0
        target.point.y = 40.0
        self.controller.target_callback(target)

        odom = Odometry()
        odom.pose.pose.position.x = 6.0
        odom.pose.pose.position.y = 8.0
        odom.pose.pose.orientation.w = 1.0
        self.controller.odom_callback(odom)

        self.assertAlmostEqual(self.controller.distance, 40.0)
        self.assertAlmostEqual(
            self.controller.desired_bearing,
            math.atan2(32.0, 24.0),
        )
        self.assertIsNotNone(self.controller.distance_time)
        self.assertIsNotNone(self.controller.bearing_time)

    def test_dry_run_guarantees_zero_when_drive_enabled(self):
        self.controller.drive_enabled = True
        self.controller.dry_run = True
        command = self.controller.make_command(0.0, "ALIGNED", 0.0)
        self.assertEqual(command.linear.x, 0.0)

    def test_stale_localization_stops(self):
        self.controller.odom_time = time.monotonic() - 10.0
        state, steering, _ = self.controller.calculate_control()
        self.assertEqual(state, "WAITING_FOR_LOCALIZATION")
        self.assertEqual(steering, 0.0)

    def test_arrival_is_confirmed_and_latched(self):
        near = Float64(data=1.0)
        for _ in range(self.controller.arrival_confirm_count):
            self.controller.distance_callback(near)
        self.assertTrue(self.controller.target_reached_latched)
        self.controller.distance_callback(Float64(data=20.0))
        state, steering, _ = self.controller.calculate_control()
        self.assertEqual(state, "TARGET_REACHED")
        self.assertEqual(steering, 0.0)
        self.controller.target_key = (0.0, 0.0)
        target = PointStamped()
        target.point.x = 1.0
        self.controller.target_callback(target)
        self.assertFalse(self.controller.target_reached_latched)

    def test_throttle_is_always_zero(self):
        self.controller.dry_run = False
        command = self.controller.make_command(0.5)
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(command.angular.z, 0.5)

    def test_dry_run_can_be_enabled_and_disabled(self):
        result = self.controller.parameter_callback([
            Parameter("dry_run", value=False),
        ])
        self.assertTrue(result.successful)
        self.assertFalse(self.controller.dry_run)
        self.assertEqual(
            self.controller.make_command(0.5).linear.x,
            0.0,
        )

    def test_drive_disabled_is_zero(self):
        self.controller.dry_run = False
        self.controller.drive_enabled = False
        command = self.controller.make_command(0.5, "ALIGNED", 0.0)
        self.assertEqual(command.linear.x, 0.0)

    def test_aligned_healthy_enabled_uses_bounded_throttle(self):
        self.controller.dry_run = False
        self.controller.drive_enabled = True
        command = self.controller.make_command(0.0, "ALIGNED", 0.0)
        self.assertEqual(command.linear.x, self.controller.test_throttle)
        self.assertLessEqual(command.linear.x, self.controller.max_test_throttle)

    def test_large_heading_error_stops_drive(self):
        self.controller.dry_run = False
        self.controller.drive_enabled = True
        command = self.controller.make_command(0.5, "TURN_RIGHT", math.radians(-30.1))
        self.assertEqual(command.linear.x, 0.0)

    def test_moderate_heading_error_uses_reduced_throttle(self):
        self.controller.dry_run = False
        self.controller.drive_enabled = True
        command = self.controller.make_command(
            0.5, "TURN_RIGHT", math.radians(-25.0)
        )
        self.assertEqual(
            command.linear.x, self.controller.turning_test_throttle
        )
        self.assertLessEqual(command.linear.x, 0.03)

    def test_missing_target_stops_drive(self):
        self.controller.dry_run = False
        self.controller.drive_enabled = True
        self.controller.target_key = None
        command = self.controller.make_command(0.0, "ALIGNED", 0.0)
        self.assertEqual(command.linear.x, 0.0)

    def test_throttle_hard_maximum(self):
        self.controller.dry_run = False
        self.controller.drive_enabled = True
        self.controller.test_throttle = self.controller.max_test_throttle
        command = self.controller.make_command(0.0, "ALIGNED", 0.0)
        self.assertLessEqual(command.linear.x, 0.10)

    def test_first_drive_limiter_latches_after_timeout(self):
        self.controller.dry_run = False
        self.controller.drive_enabled = True
        self.controller.first_drive_max_seconds = 0.01
        self.controller.make_command(0.0, "ALIGNED", 0.0)
        time.sleep(0.02)
        command = self.controller.make_command(0.0, "ALIGNED", 0.0)
        self.assertEqual(command.linear.x, 0.0)
        self.assertTrue(self.controller.first_drive_latched)


if __name__ == "__main__":
    unittest.main()
