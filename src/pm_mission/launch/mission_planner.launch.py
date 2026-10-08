"""Laptop側のミッション編集アプリだけを起動する。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    path = os.path.join(get_package_share_directory('pm_mission'), 'config', 'mission.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=path),
        Node(package='pm_mission', executable='mission_planner', output='screen',
             arguments=['--config', LaunchConfiguration('config')]),
    ])
