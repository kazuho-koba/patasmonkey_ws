"""UGV Coreと経路受信箱を起動する。recorderや経路走行は起動しない。"""
# 具体的なNode構成・launch引数・起動順は ../pm_bringup/stack_launch.py に集約する。
# このファイルは共通実装を「記録なし」で呼び出す起動入口。
from pm_bringup.stack_launch import create_launch_description
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os


def generate_launch_description():
    """既存の起動順と引数を共有し、記録プロセスを切り離す。"""
    description = create_launch_description(legacy_recording=False, drive_default=False)
    receiver_config = os.path.join(get_package_share_directory('pm_mission'),
                                   'config', 'receiver.yaml')
    # 受領・保存だけをCoreに追加する。旧一括launchとmotor/teleopの既定値は変えず、
    # 重複receiverを手動起動する場合はこの引数をfalseにできる。
    description.add_action(DeclareLaunchArgument(
        'use_mission_receiver', default_value='true', description='経路の受領・保存nodeを起動'))
    description.add_action(DeclareLaunchArgument(
        'mission_receiver_config', default_value=receiver_config,
        description='経路受信service・保存先のROS parameter YAML'))
    description.add_action(Node(
        package='pm_mission', executable='mission_receiver', name='mission_receiver',
        parameters=[LaunchConfiguration('mission_receiver_config')], output='screen',
        condition=IfCondition(LaunchConfiguration('use_mission_receiver'))))
    # 実行管理は受領経路と開始・終了要求を扱う。MPPI未接続の間は開始を拒否する。
    description.add_action(DeclareLaunchArgument('use_execution_manager', default_value='true'))
    description.add_action(Node(package='pm_planner', executable='execution_manager',
        output='screen', condition=IfCondition(LaunchConfiguration('use_execution_manager'))))
    planner_config = os.path.join(get_package_share_directory('pm_planner'), 'config', 'planner.yaml')
    # legacyはnavsatのworldがodom、それ以外はglobal EKFに合わせたmap。
    # FromLLのframeは/odometry/gpsで照合し、不一致ならPathを生成しない。
    legacy = ["'odom' if '", LaunchConfiguration('localization_mode'), "' == 'legacy' else 'map'"]
    fix = ["'/fix' if '", LaunchConfiguration('localization_mode'), "' == 'legacy' else '/fix/gated'"]
    pose = ["'/odometry/local' if '", LaunchConfiguration('localization_mode'), "' == 'legacy' else '/odometry/global'"]
    description.add_action(Node(package='pm_planner', executable='planner_preview',
        name='preview', namespace='pm/planner', parameters=[planner_config, {
            'managed_mission_enabled': True, 'planning_frame': PythonExpression(legacy),
            'navsat_world_frame': PythonExpression(legacy), 'gnss_topic': PythonExpression(fix),
            'pose_topic': PythonExpression(pose)}], output='screen',
        condition=IfCondition(LaunchConfiguration('use_execution_manager'))))
    return description
