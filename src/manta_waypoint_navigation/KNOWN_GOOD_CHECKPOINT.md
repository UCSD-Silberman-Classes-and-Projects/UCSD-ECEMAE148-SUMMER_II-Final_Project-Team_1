# Point-B Navigation Checkpoint

Validated on 2026-08-31 with ROS 2 Jazzy and `ROS_DOMAIN_ID=96`.

## Sensor and Localization Baseline

- GNSS: SparkFun NEO-F10N, `/dev/ttyUSB0`, 38400 baud, topic `/fix`.
- IMU: BNO085, `/dev/i2c-1`, address `0x4b`, topic `/imu`.
- Localization output: `/manta/fused_odom`.
- Vehicle yaw uses the validated `imu_yaw_offset_deg: -90.0`.
- GPS updates position only; BNO085 quaternion supplies absolute yaw.

## Waypoint Controller Baseline

- Output: `/manta/nav/cmd_vel_raw` → `manta_safety_gate` → `/cmd_vel`.
- `linear.x` is always `0.0`; autonomous throttle is not implemented.
- Steering: `Kp=0.8`, clamp `±0.5`, deadband `5°`.
- Arrival radius: `2.0 m`; confirmation count: `5`; arrival is latched.
- GNSS gate: maximum sigma `5.0 m`, 3 bad fixes to stop, 5 good fixes
  to recover.
- Conservative horizontal sigma is `max(sqrt(cov[0]), sqrt(cov[4]))`.
- Controller starts in dry-run. The safety gate starts stop-latched.
- VESC launch is disabled unless `launch_actuator:=true` is explicit.
- OAK-D/YOLO safety inputs are disabled unless
  `launch_safety_inputs:=true` is explicit. If absent or stale, the safety
  gate remains stop-latched.

## Verification

Eight software-only tests pass: left, right, aligned, unreliable GNSS,
stale localization, arrival latch, runtime dry-run transition, and permanent
zero throttle. Build only with `colcon build --packages-select
manta_waypoint_navigation`; never use `--symlink-install`.
