#!/usr/bin/env python3
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pm_config_dir = Path(get_package_share_directory("pm_config"))

    vehicle_geometry_yaml = pm_config_dir / "config" / "vehicle_geometry.yaml"
    vehicle_control_yaml = pm_config_dir / "config" / "vehicle_control.yaml"
    odometry_yaml = pm_config_dir / "config" / "odometry.yaml"

    wheel_odometry_node = Node(
            package="pm_localization",
            executable="wheel_odometry_node",
            name="wheel_odometry_node",
            parameters=[
                str(vehicle_geometry_yaml),
                str(vehicle_control_yaml),
                str(odometry_yaml),
            ],
            output="screen",
        )

    return LaunchDescription([
        wheel_odometry_node,
    ])