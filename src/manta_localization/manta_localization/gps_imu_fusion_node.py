#!/usr/bin/env python3

import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


EARTH_RADIUS_M = 6378137.0


def wrap_angle(angle):
    """Wrap angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class MantaGpsImuEkf(Node):
    """
    Simple 5-state EKF for a small ground vehicle.

    State:
        x[0] = local east position [m]
        x[1] = local north position [m]
        x[2] = yaw [rad]
        x[3] = forward velocity [m/s]
        x[4] = forward accelerometer bias [m/s^2]

    Inputs:
        NavSatFix:
            absolute latitude/longitude -> local east/north measurement
            (published by nmea_navsat_driver from the NEO-F10N on /fix)

        Imu:
            forward-axis linear acceleration
            z-axis yaw rate
            absolute yaw from the orientation quaternion
            (published by bno08x_driver from the BNO080 on /imu)
    """

    def __init__(self):
        super().__init__("manta_gps_imu_ekf")

        self.declare_parameter("gps_topic", "/fix")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")

        self.declare_parameter("forward_axis", "x")
        self.declare_parameter("forward_axis_sign", 1.0)
        self.declare_parameter("yaw_rate_sign", 1.0)
        self.declare_parameter("imu_yaw_sigma_deg", 5.0)
        self.declare_parameter("imu_yaw_offset_deg", 0.0)

        self.declare_parameter("gps_sigma_fallback_m", 1.5)

        self.declare_parameter("max_dt_s", 0.20)
        self.declare_parameter("path_max_points", 2000)

        # Startup stationary accelerometer calibration.
        self.declare_parameter("accel_calibration_seconds", 3.0)
        # Floor limit for standalone GNSS covariance.
        self.declare_parameter("gps_sigma_min_m", 1.0)

        self.gps_topic = self.get_parameter("gps_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        self.forward_axis = str(
            self.get_parameter("forward_axis").value
        ).lower()

        self.forward_axis_sign = float(
            self.get_parameter("forward_axis_sign").value
        )

        self.yaw_rate_sign = float(
            self.get_parameter("yaw_rate_sign").value
        )

        self.imu_yaw_sigma = math.radians(
            float(self.get_parameter("imu_yaw_sigma_deg").value)
        )

        self.imu_yaw_offset = math.radians(
            float(self.get_parameter("imu_yaw_offset_deg").value)
        )

        self.gps_sigma_fallback = float(
            self.get_parameter("gps_sigma_fallback_m").value
        )

        self.max_dt = float(
            self.get_parameter("max_dt_s").value
        )

        self.path_max_points = int(
            self.get_parameter("path_max_points").value
        )
    
        self.accel_calibration_seconds = float(
            self.get_parameter("accel_calibration_seconds").value
        )

        self.gps_sigma_min = float(
            self.get_parameter("gps_sigma_min_m").value
        )
        if self.forward_axis not in ("x", "y", "z"):
            raise ValueError(
                "forward_axis must be one of: x, y, z"
            )

        # State: [x, y, yaw, velocity, accel_bias].
        self.x = np.zeros((5, 1), dtype=float)

        self.P = np.diag([
            25.0,
            25.0,
            math.radians(90.0) ** 2,
            4.0,
            1.0,
        ])

        # Base process-noise terms.
        self.q_position = 0.05
        self.q_yaw = math.radians(8.0) ** 2
        self.q_velocity = 0.80
        self.q_bias = 0.01

        self.origin_lat_rad = None
        self.origin_lon_rad = None

        self.last_imu_time = None
        self.last_yaw_rate = 0.0
        self.yaw_initialized = False

        self.accel_calibrated = False
        self.accel_calibration_start_time = None
        self.accel_calibration_samples = []

        self.have_gps = False
        self.have_imu = False

        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            10,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            50,
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            "/manta/fused_odom",
            10,
        )

        self.gps_path_pub = self.create_publisher(
            Path,
            "/manta/path/gps",
            10,
        )

        self.fused_path_pub = self.create_publisher(
            Path,
            "/manta/path/fused",
            10,
        )

        self.gps_path_points = deque(
            maxlen=self.path_max_points
        )

        self.fused_path_points = deque(
            maxlen=self.path_max_points
        )

        self.get_logger().info(
            "MANTA GPS-IMU EKF started.\n"
            f"  GPS topic: {self.gps_topic}\n"
            f"  IMU topic: {self.imu_topic}\n"
            f"  forward_axis: {self.forward_axis}\n"
            f"  forward_axis_sign: {self.forward_axis_sign}\n"
            f"  yaw_rate_sign: {self.yaw_rate_sign}\n"
            "  imu_yaw_offset_deg: "
            f"{math.degrees(self.imu_yaw_offset):.1f}"
        )

    def latlon_to_local_xy(self, lat_deg, lon_deg):
        """
        Convert latitude/longitude to local east/north meters using an
        equirectangular approximation about the first valid GPS fix.
        """
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)

        if self.origin_lat_rad is None:
            self.origin_lat_rad = lat
            self.origin_lon_rad = lon

            self.get_logger().info(
                f"GPS origin set: "
                f"lat={lat_deg:.9f}, lon={lon_deg:.9f}"
            )

        dlat = lat - self.origin_lat_rad
        dlon = lon - self.origin_lon_rad

        x_east = (
            EARTH_RADIUS_M
            * dlon
            * math.cos(self.origin_lat_rad)
        )

        y_north = EARTH_RADIUS_M * dlat

        return x_east, y_north

    def get_forward_acceleration(self, msg):
        if self.forward_axis == "x":
            value = msg.linear_acceleration.x
        elif self.forward_axis == "y":
            value = msg.linear_acceleration.y
        else:
            value = msg.linear_acceleration.z

        return self.forward_axis_sign * float(value)

    def get_orientation_yaw(self, msg):
        quaternion = msg.orientation
        values = (
            float(quaternion.x),
            float(quaternion.y),
            float(quaternion.z),
            float(quaternion.w),
        )

        if not all(math.isfinite(value) for value in values):
            return None

        qx, qy, qz, qw = values
        norm = math.sqrt(
            qx * qx + qy * qy + qz * qz + qw * qw
        )
        if norm <= 1e-9:
            return None

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        sin_yaw = 2.0 * (qw * qz + qx * qy)
        cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
        sensor_yaw = math.atan2(sin_yaw, cos_yaw)
        return wrap_angle(sensor_yaw + self.imu_yaw_offset)

    def predict(self, accel_forward, yaw_rate, dt):
        yaw = float(self.x[2, 0])
        velocity = float(self.x[3, 0])
        accel_bias = float(self.x[4, 0])

        corrected_accel = accel_forward - accel_bias

        c = math.cos(yaw)
        s = math.sin(yaw)

        travel = (
            velocity * dt
            + 0.5 * corrected_accel * dt * dt
        )

        self.x[0, 0] += travel * c
        self.x[1, 0] += travel * s

        self.x[2, 0] = wrap_angle(
            self.x[2, 0] + yaw_rate * dt
        )

        self.x[3, 0] += corrected_accel * dt

        F = np.eye(5)

        F[0, 2] = -travel * s
        F[0, 3] = dt * c
        F[0, 4] = -0.5 * dt * dt * c

        F[1, 2] = travel * c
        F[1, 3] = dt * s
        F[1, 4] = -0.5 * dt * dt * s

        F[3, 4] = -dt

        Q = np.diag([
            self.q_position * dt,
            self.q_position * dt,
            self.q_yaw * dt,
            self.q_velocity * dt,
            self.q_bias * dt,
        ])

        self.P = F @ self.P @ F.T + Q
        self.x[2, 0] = wrap_angle(self.x[2, 0])

    def update_gps_position(
        self,
        gps_x,
        gps_y,
        sigma_x,
        sigma_y,
    ):
        z = np.array([
            [gps_x],
            [gps_y],
        ])

        H = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0],
        ])

        R = np.diag([
            max(
        sigma_x,
        self.gps_sigma_min,
            ) ** 2,
    
        max(
        sigma_y,
        self.gps_sigma_min,
        ) ** 2,
    ])

        innovation = z - H @ self.x

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation

        I = np.eye(5)
        self.P = (I - K @ H) @ self.P

        self.x[2, 0] = wrap_angle(
            self.x[2, 0]
        )

    def update_imu_yaw(
        self,
        yaw_measurement,
    ):
        H = np.array([
            [0.0, 0.0, 1.0, 0.0, 0.0]
        ])

        R = np.array([
            [self.imu_yaw_sigma ** 2]
        ])

        innovation = wrap_angle(
            yaw_measurement - float(self.x[2, 0])
        )

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K * innovation

        I = np.eye(5)
        self.P = (I - K @ H) @ self.P

        self.x[2, 0] = wrap_angle(
            self.x[2, 0]
        )

    def imu_callback(self, msg):
        timestamp = stamp_to_seconds(
            msg.header.stamp
        )

        if timestamp <= 0.0:
            timestamp = (
                self.get_clock().now().nanoseconds
                * 1e-9
            )

        accel_forward = (
            self.get_forward_acceleration(msg)
        )

        yaw_rate = (
            self.yaw_rate_sign
            * float(msg.angular_velocity.z)
        )

        orientation_yaw = self.get_orientation_yaw(msg)
        if orientation_yaw is not None and not self.yaw_initialized:
            self.x[2, 0] = orientation_yaw
            self.P[2, 2] = self.imu_yaw_sigma ** 2
            self.yaw_initialized = True

        # ---------------------------------------------------------
        # Startup accelerometer bias calibration.
        #
        # Keep the vehicle completely stationary during this period.
        # This captures mounting tilt + constant sensor bias as the
        # forward-axis acceleration bias.
        # ---------------------------------------------------------
    
        if not self.accel_calibrated:
    
            if self.accel_calibration_start_time is None:
                self.accel_calibration_start_time = timestamp 
    
                self.get_logger().info(
                    "Starting stationary accelerometer calibration. "
                    "Keep the vehicle still."
                )
            self.accel_calibration_samples.append(
                accel_forward
            )
    
            calibration_elapsed = (
                timestamp
                - self.accel_calibration_start_time
            )
    
            if (
                calibration_elapsed
                >= self.accel_calibration_seconds
            ):
                measured_bias = float(
                    np.mean(
                        self.accel_calibration_samples
                    )
                )
                
                self.x[4,0] = measured_bias
                self.accel_calibrated = True
                self.last_imu_time = timestamp
                self.get_logger().info(
                    f"Accelerometer calibration complete."
                    f"Forward bias = {measured_bias:.4f} m/s^2"
                    f"from {len(self.accel_calibration_samples)} samples."
                )
    
            return


        self.last_yaw_rate = yaw_rate
        self.have_imu = True

        if self.last_imu_time is None:
            self.last_imu_time = timestamp
            return

        dt = timestamp - self.last_imu_time
        self.last_imu_time = timestamp

        if dt <= 0.0:
            return

        if dt > self.max_dt:
            self.get_logger().warning(
                f"Large IMU dt={dt:.3f}s; "
                f"clipping to {self.max_dt:.3f}s"
            )
            dt = self.max_dt

        self.predict(
            accel_forward=accel_forward,
            yaw_rate=yaw_rate,
            dt=dt,
        )

        if orientation_yaw is not None:
            self.update_imu_yaw(orientation_yaw)

        if self.have_gps:
            self.publish_fused_state(
                msg.header.stamp
            )

    def gps_callback(self, msg):
        if (
            msg.status.status
            == NavSatStatus.STATUS_NO_FIX
        ):
            self.get_logger().warning(
                "GPS reports NO_FIX"
            )
            return

        if not math.isfinite(msg.latitude):
            return

        if not math.isfinite(msg.longitude):
            return

        gps_x, gps_y = (
            self.latlon_to_local_xy(
                msg.latitude,
                msg.longitude,
            )
        )

        sigma_x = self.gps_sigma_fallback
        sigma_y = self.gps_sigma_fallback

        cov = list(msg.position_covariance)

        if len(cov) == 9:
            if (
                cov[0] > 0.0
                and math.isfinite(cov[0])
            ):
                sigma_x = math.sqrt(cov[0])

            if (
                cov[4] > 0.0
                and math.isfinite(cov[4])
            ):
                sigma_y = math.sqrt(cov[4])

        self.update_gps_position(
            gps_x=gps_x,
            gps_y=gps_y,
            sigma_x=sigma_x,
            sigma_y=sigma_y,
        )

        self.have_gps = True

        self.publish_gps_path(
            gps_x,
            gps_y,
            msg.header.stamp,
        )

        self.publish_fused_state(
            msg.header.stamp
        )

    def yaw_to_quaternion_zw(self, yaw):
        half = 0.5 * yaw
        qz = math.sin(half)
        qw = math.cos(half)
        return qz, qw

    def publish_fused_state(self, stamp):
        yaw = float(self.x[2, 0])

        qz, qw = self.yaw_to_quaternion_zw(
            yaw
        )

        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = (
            self.child_frame_id
        )

        odom.pose.pose.position.x = float(
            self.x[0, 0]
        )

        odom.pose.pose.position.y = float(
            self.x[1, 0]
        )

        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(
            self.x[3, 0]
        )

        odom.twist.twist.angular.z = (
            self.last_yaw_rate
        )

        pose_cov = [0.0] * 36
        pose_cov[0] = float(self.P[0, 0])
        pose_cov[7] = float(self.P[1, 1])
        pose_cov[35] = float(self.P[2, 2])
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = float(self.P[3, 3])
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose

        self.fused_path_points.append(pose)

        path = Path()
        path.header = odom.header
        path.poses = list(
            self.fused_path_points
        )

        self.fused_path_pub.publish(path)

    def publish_gps_path(
        self,
        gps_x,
        gps_y,
        stamp,
    ):
        pose = PoseStamped()

        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id

        pose.pose.position.x = float(gps_x)
        pose.pose.position.y = float(gps_y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        self.gps_path_points.append(pose)

        path = Path()
        path.header = pose.header
        path.poses = list(
            self.gps_path_points
        )

        self.gps_path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)

    node = MantaGpsImuEkf()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
