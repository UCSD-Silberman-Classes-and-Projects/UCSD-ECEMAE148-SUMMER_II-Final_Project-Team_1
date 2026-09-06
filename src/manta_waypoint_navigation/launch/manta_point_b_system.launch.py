import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bno_launch = os.path.join(
        get_package_share_directory("bno08x_driver"),
        "launch",
        "bno085_i2c.launch.py",
    )
    localization_launch = os.path.join(
        get_package_share_directory("manta_localization"),
        "launch",
        "fusion.launch.py",
    )
    actuator_launch = os.path.join(
        get_package_share_directory("ucsd_robocar_actuator2_pkg"),
        "launch",
        "vesc_twist.launch.py",
    )
    camera_safety_launch = os.path.join(
        get_package_share_directory("ucsd_robocar_sensor2_pkg"),
        "launch",
        "camera_oakd.launch.py",
    )

    dry_run = LaunchConfiguration("dry_run")
    launch_actuator = LaunchConfiguration("launch_actuator")
    launch_safety_inputs = LaunchConfiguration("launch_safety_inputs")

    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("launch_actuator", default_value="false"),
        DeclareLaunchArgument(
            "launch_safety_inputs",
            default_value="false",
        ),
        LogInfo(msg=(
            "KEEP CAR STATIONARY: localization performs a 3-second "
            "accelerometer calibration after startup."
        )),
        Node(
            package="nmea_navsat_driver",
            executable="nmea_serial_driver",
            name="manta_gps_driver",
            output="screen",
            parameters=[{
                "port": "/dev/ttyUSB0",
                "baud": 38400,
                "frame_id": "gps_link",
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bno_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_safety_launch),
            condition=IfCondition(launch_safety_inputs),
        ),
        TimerAction(
            period=2.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
            )],
        ),
        Node(
            package="ucsd_robocar_control2_pkg",
            executable="manta_safety_gate",
            name="manta_safety_gate",
            output="screen",
        ),
        TimerAction(
            period=6.0,
            actions=[Node(
                package="manta_waypoint_navigation",
                executable="waypoint_controller",
                name="waypoint_controller",
                output="screen",
                parameters=[{"dry_run": dry_run}],
            )],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(actuator_launch),
            condition=IfCondition(launch_actuator),
        ),
    ])
