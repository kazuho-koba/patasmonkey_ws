#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # patasmonkey_teleop パッケージの launch
    teleop_launch_file = (
        Path(get_package_share_directory("patasmonkey_teleop"))
        / "launch"
        / "joy_teleop.launch.py"
    )

    # patasmonkey_vehicle_interface パッケージの launch
    vehicle_launch_file = (
        Path(get_package_share_directory("patasmonkey_vehicle_interface"))
        / "launch"
        / "vehicle_interface.launch.py"
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(teleop_launch_file))
    )
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(vehicle_launch_file))
    )

    return LaunchDescription(
        [
            teleop_launch,
            vehicle_launch,
        ]
    )
