"""obstacle閾値・セル幅を編集して、最新localizationとRVizでbagを確認する。

処理内容は既存terrain_mapping_latest_localization_replay.launch.pyに委譲する。
このlaunchはセンサ・モータを起動せず、bag再生も別ターミナルで行う。
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
    # 基本条件を過去の比較baselineへ固定し、編集用YAMLで必要な値だけ上書きする。
    return LaunchDescription([
        *declare_tuning_arguments(),
        DeclareLaunchArgument(
            "review_config",
            default_value=str(share / "config" / "terrain_obstacle_review.yaml"),
            description="セル幅・obstacle閾値・QoSの編集用YAML（絶対パス推奨）。",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                share / "launch" / "terrain_mapping_latest_localization_replay.launch.py"
            )),
            launch_arguments={
                "terrain_tuning_enabled": "true",
                **{name: LaunchConfiguration(name) for name in TUNING_PARAMETERS},
                "terrain_mapper_config": str(
                    share / "config" / "depth_elevation_mapper_hazard_0p05_baseline.yaml"
                ),
                "terrain_mapper_override_config": LaunchConfiguration("review_config"),
            }.items(),
        ),
        # 同じ/clockを使うRViz。bag開始前のodom未存在警告は再生後に解消する。
        Node(
            package="rviz2", executable="rviz2", name="terrain_review_rviz",
            arguments=["-d", str(share / "rviz" / "terrain_obstacle_review.rviz")],
            parameters=[{"use_sim_time": True}], output="screen",
        ),
    ])
