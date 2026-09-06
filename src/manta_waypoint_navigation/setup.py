import os
from glob import glob

from setuptools import find_packages, setup


package_name = "manta_waypoint_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    test_suite="test",
    zip_safe=True,
    maintainer="MANTA Team 1",
    maintainer_email="noreply@example.com",
    description="Reporting-only GPS waypoint solution for MANTA.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "waypoint_cli = "
            "manta_waypoint_navigation.waypoint_cli:main",
            "waypoint_controller = "
            "manta_waypoint_navigation.waypoint_controller:main",
            "gps_sigma_monitor = "
            "manta_waypoint_navigation.gps_sigma_monitor:main",
        ],
    },
)
