"""mission/debug recorderの共通launch定義。runtime制御ノードは起動しない。"""
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def create_bag_launch(profile):
    """topic profileを選択し、出力先と再現情報の対象workspaceを明示する。"""
    def start(context):
        values = {key: LaunchConfiguration(key).perform(context) for key in (
            'bag_directory', 'bag_name', 'workspace', 'external_workspace', 'storage',
            'mission_bag', 'status_file')}
        if not values['bag_name'] or Path(values['bag_name']).name != values['bag_name']:
            raise RuntimeError('bag_nameは空でない単一のディレクトリ名を指定してください')
        output = Path(values['bag_directory']).expanduser() / values['bag_name']
        if output.exists():
            raise RuntimeError('既存bagは上書きしません: '+str(output))
        # ros2 runの中間processを挟むとFoxyのSIGINTが実record_trialへ届かず
        # 孤児化する場合があるため、launchが実processを直接監督する。
        executable = Path(get_package_prefix('pm_bringup'))/'lib/pm_bringup/record_trial'
        return [ExecuteProcess(cmd=[
            str(executable), '--profile', profile,
            '--output', str(output), '--workspace', values['workspace'],
            '--external-workspace', values['external_workspace'], '--storage', values['storage'],
            '--mission-bag', values['mission_bag'], '--status-file', values['status_file'],
        ], output='screen', sigterm_timeout='120', sigkill_timeout='120')]
    # 既知の開発コンテナmountを優先し、Jetsonホストでは従来のhome配下へ保存する。
    workspace = (Path('/workspaces/patasmonkey_ws') if Path('/workspaces/patasmonkey_ws/src').is_dir()
                 else Path.home()/'patasmonkey_ws')
    external = (Path('/workspaces/ros2_ws') if Path('/workspaces/ros2_ws/src').is_dir()
                else Path.home()/'ros2_ws')
    root = workspace / 'bags'
    if profile == 'debug':
        root = root / 'debug'
    return LaunchDescription([
        DeclareLaunchArgument('bag_directory', default_value=str(root)),
        DeclareLaunchArgument('bag_name', default_value='rosbag2_'+datetime.now().strftime('%Y_%m_%d-%H_%M_%S')),
        DeclareLaunchArgument('workspace', default_value=str(workspace)),
        DeclareLaunchArgument('external_workspace', default_value=str(external)),
        DeclareLaunchArgument('storage', default_value='mcap'),
        DeclareLaunchArgument('mission_bag', default_value='',
                              description='debug bagと対応するmission bagの実パス（任意）'),
        DeclareLaunchArgument(
            'status_file',
            default_value=str(Path.home()/'.ros'/'pm_robot_manager'/
                              (profile+'_bag_status.json')),
            description='systemd ExecStopと共有するbag停止状態ファイル'),
        OpaqueFunction(function=start),
    ])
