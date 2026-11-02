"""PC側の独立した検査画面。解析はユーザ要求後にのみ開始する。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('pm_inspector'))/'config/inspector.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='pm_inspector', executable='bag_inspector', output='screen',
             arguments=['--config', LaunchConfiguration('config')])])
