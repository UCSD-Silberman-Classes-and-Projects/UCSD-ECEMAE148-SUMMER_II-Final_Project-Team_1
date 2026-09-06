# MANTA(ray) System Breakdown

## 1. GPS

**Role:** Global position measurement.

The GPS receiver provides latitude, longitude, altitude, and position covariance through ROS 2.

Primary topic:

- /fix

Typical update rate during testing: approximately 10 Hz.

The GPS data provides the absolute position reference used by the localization and waypoint-navigation systems.

---

## 2. BNO085 IMU

**Role:** High-rate vehicle motion sensing.

The BNO085 provides:

- Orientation
- Angular velocity
- Linear acceleration

Primary topic:

- /imu

The IMU was operated at approximately 100 Hz and provides higher-rate motion information between GPS updates.

---

## 3. GPS-IMU Sensor Fusion

**Role:** Produce a higher-rate localization estimate for autonomous navigation.

ROS 2 package:

- src/manta_localization/

The fusion system combines the global position information from GPS with high-rate IMU measurements.

During testing, the localization stack maintained approximately 100 Hz output and achieved roughly 2-4 meter positioning accuracy.

Major functions include:

- GPS coordinate processing
- IMU orientation and acceleration processing
- Sensor calibration
- Motion estimation
- GPS uncertainty monitoring
- Fused localization output

---

## 4. Waypoint Navigation

**Role:** Drive the RoboCar toward a user-specified GPS coordinate.

ROS 2 package:

- src/manta_waypoint_navigation/

The navigation system accepts a destination coordinate and compares it with the current localization estimate.

It calculates:

- Remaining distance
- Target bearing
- Heading error
- Forward velocity command
- Steering command

The navigation stack successfully commanded vehicle movement through the MANTA safety chain during outdoor testing.

Point-to-point navigation reached a semi-autonomous functional state.

---

## 5. OAK-D Lite Perception

**Role:** Visual object and cone detection.

The OAK-D Lite runs a YOLO-based detection pipeline.

Important topics include:

- /cone/count_0
- /cone/detections_0
- /manta/vision/heartbeat
- /manta/vision/hazard_state

During final testing, the system successfully produced two-cone bounding-box detections at approximately 10 Hz.

The perception information was intended to support obstacle identification and visual guidance through a cone-defined path.

---

## 6. LD06 LiDAR

**Role:** Independent forward obstacle monitoring.

The LD06 provides range measurements used by the safety layer to determine whether the vehicle has sufficient forward clearance.

The LiDAR was successfully integrated with the MANTA safety system.

---

## 7. MANTA Safety Gate

**Role:** Final software authority over vehicle motion.

Navigation and avoidance systems generate requested motion commands upstream of the safety gate.

The safety gate evaluates:

- LiDAR health
- Forward obstacle distance
- Localization health
- Navigation command freshness
- Vision / maneuver state

Only commands accepted by the safety layer are forwarded toward the vehicle actuator interface.

This architecture allows the safety system to stop the RoboCar independently of the active navigation controller.

---

## 8. Vehicle Actuation / VESC

**Role:** Convert final ROS 2 velocity commands into physical steering and propulsion.

The VESC and vehicle actuator stack are part of the UCSD RoboCar platform and are treated as external dependencies rather than Team 1 source code.

The MANTA stack interfaces with the actuator system only after commands pass through the safety gate.

---

## 9. Runtime / Orchestration

**Role:** Start, monitor, arm, and stop the integrated MANTA system.

Located under:

- runtime/

Important utilities include:

- manta-up
- manta-arm
- manta-status
- manta-stop
- manta-log
- manta_lidar_health_stabilizer.py

These tools coordinate the project-specific ROS 2 components without modifying the underlying UCSD RoboCar platform.

---

## 10. Experimental Integration Layer

Located under:

- experimental/

This directory contains approaches tested for:

- Two-cone visual guidance
- Command arbitration
- Avoidance-state management
- Direct OAK-D detection
- Navigation handoff

These files document the development process but are not all part of the final validated runtime configuration.

---

## Full Data Flow

GPS + IMU
    -> GPS-IMU Fusion
    -> Fused Localization
    -> Waypoint Navigation
    -> Command Arbitration
    -> Safety Gate
    -> VESC
    -> RoboCar

OAK-D Lite
    -> YOLO Detection
    -> Visual Avoidance
    -> Command Arbitration

LD06 LiDAR
    -> Safety Monitoring
    -> Safety Gate

---

## Main Integration Challenge

The individual subsystems were largely functional, but full-stack operation required reliable handoffs between perception, avoidance, waypoint navigation, and the safety gate.

During live testing, duplicated publishers and state-transition problems could cause stopping behavior or unstable steering.

The preferred future architecture would use one explicit state machine and guarantee exactly one active motion-command publisher.
