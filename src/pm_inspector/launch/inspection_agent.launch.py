"""Jetson側の受付だけを起動し、Coreや記録は操作しない。"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_root', default_value=str(Path.home()/'patasmonkey_ws/bags')),
        DeclareLaunchArgument('result_root', default_value=str(Path.home()/'.ros/pm_inspector')),
        DeclareLaunchArgument('service', default_value='/pm/inspector/request'),
        Node(package='pm_inspector', executable='inspection_agent', output='screen',
             parameters=[{'bag_root': LaunchConfiguration('bag_root'),
                          'result_root': LaunchConfiguration('result_root'),
                          'service': LaunchConfiguration('service')}])])
