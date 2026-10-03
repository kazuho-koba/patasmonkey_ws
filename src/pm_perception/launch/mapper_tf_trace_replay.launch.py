"""通常mapperの使用TFと評価時刻を採取する、別建てのoffline replay launch。

同じ再生の/odometry/localは専用CSV collectorで保存する。センサ・モータは
起動しない。既存RViz用launch・YAMLの既定動作は変更しない。
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pm_perception.terrain_launch_parameters import declare_tuning_arguments, terrain_tuning_overrides


def generate_launch_description():
    share = Path(get_package_share_directory("pm_perception"))
    def mapper_node(context):
        # YAML優先順を維持し、CLI上書きと閾値に一致する表示スケールを最後に適用。
        paths = [LaunchConfiguration("mapper_config").perform(context),
                 str(share/"config"/"depth_elevation_mapper_old_bag.yaml"),
                 LaunchConfiguration("trace_config").perform(context)]
        return [Node(package="pm_perception", executable="depth_elevation_mapper_node",
                     name="depth_elevation_mapper", output="screen", parameters=[
                         terrain_tuning_overrides(context, paths),
                         {"use_sim_time": True,
                          "replay_trace_output_dir": LaunchConfiguration("trace_output_dir")}])]
    return LaunchDescription([
        *declare_tuning_arguments(),
        DeclareLaunchArgument("trace_output_dir", description="mapper traceの新規保存ディレクトリ"),
        DeclareLaunchArgument("mapper_config", default_value=str(
            share/"config"/"depth_elevation_mapper_hazard_0p05_baseline.yaml")),
        DeclareLaunchArgument("trace_config", default_value=str(share/"config"/"mapper_tf_trace.yaml")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            share/"launch"/"single_frame_localization_replay.launch.py"))),
        OpaqueFunction(function=mapper_node),
    ])
