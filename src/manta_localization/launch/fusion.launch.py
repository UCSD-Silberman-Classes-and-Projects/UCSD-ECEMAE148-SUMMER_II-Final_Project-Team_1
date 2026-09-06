from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory,
)

import os


def generate_launch_description():
    pkg_share = get_package_share_directory(
        "manta_localization"
    )

    param_file = os.path.join(
        pkg_share,
        "config",
        "fusion.yaml",
    )

    return LaunchDescription([
        Node(
            package="manta_localization",
            executable="gps_imu_fusion",
            name="manta_gps_imu_ekf",
            output="screen",
            parameters=[param_file],
        )
    ])
