"""経路受領・参照Path・実行管理を起動。詳細はREADMEとconfig/planner.yaml参照。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('pm_planner'))
    sim=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    common={'use_sim_time':sim,'pose_topic':LaunchConfiguration('pose_topic')}
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(share/'config/planner.yaml')),
        DeclareLaunchArgument('pose_topic',default_value='/odometry/global'),
        DeclareLaunchArgument('planning_frame',default_value='map'),
        DeclareLaunchArgument('navsat_world_frame',default_value='map'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('use_mission_receiver',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='false'),
        Node(package='pm_mission',executable='mission_receiver',
             parameters=[str(Path(get_package_share_directory('pm_mission'))/'config/receiver.yaml')],
             condition=IfCondition(LaunchConfiguration('use_mission_receiver')),output='screen'),
        Node(package='pm_planner',executable='planner_preview',parameters=[LaunchConfiguration('config'),common,
             {'managed_mission_enabled':True,'planning_frame':LaunchConfiguration('planning_frame'),
              'navsat_world_frame':LaunchConfiguration('navsat_world_frame')}],output='screen'),
        Node(package='pm_planner',executable='execution_manager',parameters=[LaunchConfiguration('config'),common],output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/planner.rviz')],
             parameters=[{'use_sim_time':sim}],condition=IfCondition(LaunchConfiguration('rviz')))])
