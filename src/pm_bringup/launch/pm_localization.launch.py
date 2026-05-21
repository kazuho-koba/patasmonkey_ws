#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 各種パッケージのパス
    pm_teleop_share = Path(get_package_share_directory("pm_teleop"))
    pm_vehicle_share = Path(
        get_package_share_directory("pm_vehicle_interface"))
    pm_description_share = Path(get_package_share_directory("pm_description"))
    pm_config_share = Path(get_package_share_directory("pm_config"))

    # 既存launchファイル
    teleop_launch_file = pm_teleop_share/"launch"/"joy_teleop.launch.py"
    vehicle_launch_file = pm_vehicle_share/"launch"/"vehicle_interface.launch.py"

    # configファイル等
    urdf_file = pm_description_share/"urdf"/"pm.urdf"
    imu_config_file = pm_config_share/"config"/"hwt905_imu.yaml"
    ekf_config_file = pm_config_share/"config"/"ekf_wheel_imu.yaml"

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(teleop_launch_file)))
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(vehicle_launch_file)))

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, }],
    )
    imu_node = Node(
        package="hwt905_rs485_driver",
        executable="hwt905_rs485_driver",
        name="hwt905_imu_node",
        output="screen",
        parameters=[str(imu_config_file)],
    )
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[str(ekf_config_file)],
    )

    return LaunchDescription(
        [
            teleop_launch,
            vehicle_launch,
            robot_state_publisher_node,
            imu_node,
            ekf_node,
        ]
    )
