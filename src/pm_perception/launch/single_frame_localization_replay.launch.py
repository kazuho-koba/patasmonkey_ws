"""独立frame診断用：最新localizationだけをbag入力から再計算する。

既存の融合・RViz用launchを変更せずに使い分けるための専用launch。
mapper・実センサ・モータ・bag記録は起動しない。出力/odometry/localを
record_recomputed_local_poses.pyで保存し、独立frame評価の姿勢補間に用いる。
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("pm_config"))/"config"/"ekf_local_horizontal_vio_twist.yaml"
    sim_time = {"use_sim_time": True}
    # 最新localization replayと同じgate・水平EKF・姿勢高さobserver・composer構成。
    return LaunchDescription([
        Node(package="pm_localization", executable="vio_vertical_gate_node",
             name="vio_vertical_gate_node", parameters=[sim_time], output="screen"),
        Node(package="pm_localization", executable="vio_twist_gate_node",
             name="vio_twist_gate_node", parameters=[sim_time], output="screen"),
        Node(package="robot_localization", executable="ekf_node",
             name="ekf_local_horizontal_node", parameters=[str(config), sim_time],
             remappings=[("odometry/filtered", "/odometry/local_horizontal")], output="screen"),
        Node(package="pm_localization", executable="attitude_height_observer_node",
             name="terrain_replay_attitude_height_observer",
             parameters=[sim_time, {"output_topic": "/odometry/local_vertical"}], output="screen"),
        Node(package="pm_localization", executable="local_odometry_composer_node",
             name="terrain_replay_local_odometry_composer",
             parameters=[sim_time, {"horizontal_topic": "/odometry/local_horizontal",
                                    "vertical_topic": "/odometry/local_vertical",
                                    "output_topic": "/odometry/local", "publish_tf": True}],
             output="screen"),
    ])
