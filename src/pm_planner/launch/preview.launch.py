"""非駆動previewのみ起動する。core・センサ・perception・motorは起動しない。

処理内容と座標契約・使用例はREADME.md、具体的な既定設定はconfig/planner.yaml参照。
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('pm_planner'))
    names=dict(mission_file='',coordinate_mode='',anchor_file='',planning_frame='',
               navsat_world_frame='',pose_topic='')
    arguments=[DeclareLaunchArgument('config',default_value=str(share/'config/planner.yaml')),
               DeclareLaunchArgument('rviz',default_value='false'),
               DeclareLaunchArgument('use_sim_time',default_value='false')]
    arguments.extend(DeclareLaunchArgument(k,default_value=v) for k,v in names.items())
    sim_time=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    def preview(context):
        # 未指定引数でYAMLの値を上書きしない。明示CLI引数だけを優先する。
        parameters={k:LaunchConfiguration(k).perform(context) for k in names
                    if LaunchConfiguration(k).perform(context)}
        parameters['use_sim_time']=sim_time
        return [Node(package='pm_planner',executable='planner_preview',name='preview',namespace='pm/planner',
                     parameters=[LaunchConfiguration('config'),parameters],output='screen')]
    return LaunchDescription(arguments+[OpaqueFunction(function=preview),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/preview.rviz')],
             parameters=[{'use_sim_time':sim_time}],condition=IfCondition(LaunchConfiguration('rviz')))])
