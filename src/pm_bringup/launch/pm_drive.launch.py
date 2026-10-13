"""Coreと独立した走行許可。joy入力はCore側を再利用する。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, RegisterEventHandler, EmitEvent,
    DeclareLaunchArgument, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 片方のnodeが終了したら残りも停止し、制御経路の部分稼働を残さない。
    teleop = Path(get_package_share_directory('pm_teleop')) / 'launch/joy_teleop.launch.py'
    vehicle = Path(get_package_share_directory('pm_control')) / 'launch/vehicle_interface.launch.py'
    control_profile = Path(get_package_share_directory('pm_robot_manager')) / 'config/control_dds_udp.xml'
    return LaunchDescription([
        # Coreの画像/recorderとは別process。小さいstatus・速度指令はUDP経由にし、
        # SHM破損による「接続済みなのにManagerが確認できない」状態を避ける。
        DeclareLaunchArgument('drive_dds_profile', default_value=str(control_profile)),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                               LaunchConfiguration('drive_dds_profile')),
        RegisterEventHandler(OnProcessExit(on_exit=[
            EmitEvent(event=Shutdown(reason='走行用nodeが終了しました'))])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(teleop)),
                                 launch_arguments={'use_joy': 'false', 'use_teleop': 'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vehicle)),
                                 launch_arguments={'require_neutral_on_start': 'true'}.items()),
    ])
