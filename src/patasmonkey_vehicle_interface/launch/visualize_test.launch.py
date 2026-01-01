#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ファイルパス
    desc_share = get_package_share_directory("patasmonkey_description")
    vi_share = get_package_share_directory("patasmonkey_vehicle_interface")

    # URDFファイル
    default_urdf = os.path.join(desc_share, "urdf", "patasmonkey.urdf")

    # rvizの設定
    default_rviz = os.path.join(desc_share, "rviz", "display.rviz")

    # パラメータ
    bridge_params = os.path.join(vi_share, "config", "joint_state_bridge.yaml")
    vehicle_params = os.path.join(vi_share, "config", "vehicle_params.yaml")

    # robot descriptionのロード
    # URDFをそのまま文字列でrobot_state_publisherに渡す
    # (xacroの場合は別途commandで処理が必要だがいまはURDF前提)
    robot_description = ""
    if os.path.exists(default_urdf):
        with open(default_urdf, "r", encoding="utf-8") as f:
            robot_description = f.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "model",
                default_value=default_urdf,
                description="Ablosute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                "rvizconfig",
                default_value=default_rviz,
                description="Absolute path to rviz config file",
            ),
            # ダミーのホイール情報出力
            Node(
                package="patasmonkey_vehicle_interface",
                executable="dummy_jointstate_pub",
                name="dummy_jointstate_pub",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"left_cycle_per_sec": 1.0},
                    {"right_cycle_per_sec": 0.5},
                ],
            ),
            # エンコーダ情報（タイヤ通算回転数[rad]）を４輪ジョイントへ展開して/joint_statesを生成
            Node(
                package="patasmonkey_vehicle_interface",
                executable="joint_state_bridge_node",
                name="joint_state_bridge",
                output="screen",
                parameters=[bridge_params, {"use_sim_time": use_sim_time}],
            ),
            # オドメトリ計算
            Node(
                package="patasmonkey_vehicle_interface",
                executable="wheel_odometry_node",
                name="wheel_odometry_node",
                output="screen",
                parameters=[vehicle_params, {"use_sim_time": use_sim_time}],
            ),
            # robot_state_publisher (/joint_statesを購読してTFを出し、rvizでモデルが動く)
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"robot_description": robot_description},
                ],
            ),
            # rviz
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
