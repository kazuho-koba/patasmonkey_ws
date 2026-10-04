"""独立画像の地形をRVizで連続表示する。具体的なnode構成は共通replayへ委譲する。

bag再生は通常fusionのreviewと同じ別ターミナル。センサ・モータは起動しない。
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pm_perception.terrain_launch_parameters import TUNING_PARAMETERS, declare_tuning_arguments


def generate_launch_description():
    share = Path(get_package_share_directory("pm_perception"))
    return LaunchDescription([
        *declare_tuning_arguments(),
        DeclareLaunchArgument("review_config", default_value=str(
            share / "config" / "single_frame_terrain_review.yaml")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            share / "launch" / "terrain_mapping_latest_localization_replay.launch.py")),
            launch_arguments={
                "terrain_mapper_executable": "single_frame_mapper_node",
                "terrain_tuning_enabled": "true",
                "terrain_mapper_config": str(share / "config" / "depth_elevation_mapper_hazard_0p05_baseline.yaml"),
                "terrain_mapper_override_config": LaunchConfiguration("review_config"),
                **{name: LaunchConfiguration(name) for name in TUNING_PARAMETERS},
            }.items()),
        Node(package="rviz2", executable="rviz2", name="single_frame_review_rviz",
             arguments=["-d", str(share / "rviz" / "terrain_obstacle_review.rviz")],
             parameters=[{"use_sim_time": True}], output="screen"),
    ])
