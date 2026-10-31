"""MPPI非駆動nodeだけを起動。設定はconfig/mppi_poc.yamlとREADME参照。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config=str(Path(get_package_share_directory('pm_planner'))/'config/mppi_poc.yaml')
    rviz=str(Path(get_package_share_directory('pm_planner'))/'rviz/mppi_poc.rviz')
    sim_time=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    return LaunchDescription([DeclareLaunchArgument('config',default_value=config),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        Node(package='pm_planner',executable='mppi_poc',
             parameters=[LaunchConfiguration('config'),{'use_sim_time':sim_time}],output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',rviz],
             parameters=[{'use_sim_time':sim_time}],condition=IfCondition(LaunchConfiguration('rviz')))])
