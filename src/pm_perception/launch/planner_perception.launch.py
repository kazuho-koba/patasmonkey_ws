"""新producerの単独起動。従来mapperと同時には起動しない。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('pm_perception'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=str(share/'config/planner_perception.yaml')),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        Node(package='pm_perception',executable='planner_mapper_node',name='depth_elevation_mapper',
             output='screen',parameters=[LaunchConfiguration('params_file'),
             {'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}])])
