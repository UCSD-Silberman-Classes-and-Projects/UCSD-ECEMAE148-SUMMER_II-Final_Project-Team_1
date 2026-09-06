from setuptools import find_packages, setup
from glob import glob
import os

package_name = "manta_localization"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=[
        "setuptools",
        "numpy",
    ],
    zip_safe=True,
    maintainer="MANTA Team 1",
    maintainer_email="noreply@example.com",
    description=(
        "GPS-IMU sensor fusion "
        "for the MANTA RoboCar."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gps_imu_fusion = "
            "manta_localization."
            "gps_imu_fusion_node:main",
        ],
    },
)
