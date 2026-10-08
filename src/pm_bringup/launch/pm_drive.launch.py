"""Coreと独立した走行許可。joy入力はCore側を再利用する。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 片方のnodeが終了したら残りも停止し、制御経路の部分稼働を残さない。
    teleop = Path(get_package_share_directory('pm_teleop')) / 'launch/joy_teleop.launch.py'
    vehicle = Path(get_package_share_directory('pm_control')) / 'launch/vehicle_interface.launch.py'
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=[
            EmitEvent(event=Shutdown(reason='走行用nodeが終了しました'))])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(teleop)),
                                 launch_arguments={'use_joy': 'false', 'use_teleop': 'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vehicle)),
                                 launch_arguments={'require_neutral_on_start': 'true'}.items()),
    ])
