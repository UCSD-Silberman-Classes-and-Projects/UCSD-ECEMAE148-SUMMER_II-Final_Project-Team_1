# MANTA(ray)

## Multi-Sensor Autonomous Navigation & Transit Architecture

**UC San Diego MAE/ECE 148 - Summer Session II 2026 - Team 1**

MANTA(ray) is a multi-sensor autonomous RoboCar project combining GPS-IMU sensor fusion, autonomous GPS waypoint navigation, OAK-D Lite perception, and LiDAR-based safety monitoring.

## Team

- Austin - MAE
- Luis - MAE

## Project Goals

### Must-Haves

- GPS-IMU sensor fusion
- Autonomous point-to-point navigation
- OAK-D Lite YOLO object identification
- OAK-D Lite perception integration
- Obstacle avoidance software

### Nice-to-Haves

- LiDAR emergency-stop integration
- Robust recovery after obstacle avoidance
- Improved localization accuracy
- Taxi-style destination selection

## Major Results

### GPS-IMU Sensor Fusion
Functional ROS 2 localization package combining GPS and IMU measurements.

Source: src/manta_localization/

### Autonomous Point-to-Point Navigation
Functional GPS waypoint navigation package. Outdoor point-to-point navigation was successfully demonstrated.

Source: src/manta_waypoint_navigation/

### OAK-D Lite Perception
Real-time YOLO cone detection was demonstrated at approximately 10 Hz during final integration testing.

### LiDAR Safety
LiDAR obstacle monitoring was incorporated into the downstream MANTA safety architecture.

## Integration Challenges

The primary remaining challenge was reliable closed-loop integration between GPS waypoint navigation, visual obstacle guidance, LiDAR safety logic, and command arbitration.

The individual sensing and navigation components worked, while reliable autonomous handoff between GPS navigation and vision-based obstacle avoidance remained incomplete.

## If We Had Another Week

1. Complete closed-loop obstacle avoidance
2. Consolidate navigation-state arbitration
3. Improve recovery after camera-detection loss
4. Conduct additional outdoor waypoint trials
5. Quantify GPS-only versus fused localization performance
6. Consolidate experimental nodes into one tested architecture
7. Perform full end-to-end regression testing

## Repository Structure

- src/manta_localization - GPS-IMU fusion ROS 2 package
- src/manta_waypoint_navigation - GPS waypoint navigation ROS 2 package
- runtime - MANTA startup and orchestration tools
- experimental - Experimental integration utilities
- docs - Project documentation
- media - Images and demonstration media
- presentation - Final presentation material

## Dependencies

- ROS 2 Jazzy
- UCSD RoboCar ROS 2 framework
- bno08x-ros2-driver
- nmea_navsat_driver
- DepthAI / OAK-D Lite
- LD06 LiDAR ROS 2 support
- VESC actuator interface

## Course

UCSD MAE/ECE 148 - Summer Session II 2026 - Team 1
