"""home.yamlを使う非駆動MPPIデモ。詳細な設定はREADMEとconfigを参照。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_nodes(context):
    # 空引数は上書きしないため、通常はYAML内の設定値が有効になる。
    overrides = {}
    for name in ('mission_file', 'anchor_file', 'time_scale'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = float(value) if name == 'time_scale' else value
    rviz = str(Path(get_package_share_directory('pm_planner')) / 'rviz/mppi_live_demo.rviz')
    return [Node(package='pm_planner', executable='mppi_live_demo', output='screen',
                 parameters=[LaunchConfiguration('config'), overrides]),
            Node(package='rviz2', executable='rviz2', arguments=['-d', rviz],
                 condition=IfCondition(LaunchConfiguration('rviz')))]


def generate_launch_description():
    config = str(Path(get_package_share_directory('pm_planner')) / 'config/mppi_live_demo.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('mission_file', default_value=''),
        DeclareLaunchArgument('anchor_file', default_value=''),
        DeclareLaunchArgument('time_scale', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=make_nodes)])
