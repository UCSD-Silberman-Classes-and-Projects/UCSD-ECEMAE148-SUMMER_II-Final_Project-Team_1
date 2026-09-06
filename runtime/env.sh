#!/bin/bash

cd /home/projects/ros2_ws

source /opt/ros/jazzy/setup.bash
source /home/projects/sensor2_ws/src/lidars/ld06/ros2/install/setup.bash
source /home/projects/sensor2_ws/install/setup.bash
source /home/projects/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=96
export ROS_LOG_DIR=/tmp/manta_ros_logs

export MANTA_DIR=/tmp/manta
export MANTA_PID_DIR=/tmp/manta/pids

mkdir -p "$MANTA_DIR"
mkdir -p "$MANTA_PID_DIR"

# MANTA integration: avoid stale Fast DDS shared-memory locks.
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
