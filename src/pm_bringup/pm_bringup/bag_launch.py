"""mission/debug recorderの共通launch定義。runtime制御ノードは起動しない。"""
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from pm_bringup.dds_transport import launch_transport_actions


def create_bag_launch(profile):
    """topic profileを選択し、出力先と再現情報の対象workspaceを明示する。"""
    def start(context):
        values = {key: LaunchConfiguration(key).perform(context) for key in (
            'bag_directory', 'bag_name', 'workspace', 'external_workspace', 'storage',
            'mission_bag', 'status_file', 'polling_interval_ms', 'qos_profile_overrides_path',
            'max_bag_size_bytes', 'imu_sidecar', 'bulk_sidecar', 'max_cache_messages',
            'writer_backend', 'writer_queue_bytes', 'writer_queue_items', 'mcap_preset_profile')}
        if values['imu_sidecar'].lower() not in ('true', 'false'):
            raise RuntimeError('imu_sidecarはtrueまたはfalseを指定してください')
        if profile != 'mission' and values['imu_sidecar'].lower() == 'true':
            raise RuntimeError('imu_sidecarはmission専用です')
        if values['bulk_sidecar'].lower() not in ('true', 'false'):
            raise RuntimeError('bulk_sidecarはtrueまたはfalseを指定してください')
        if values['bulk_sidecar'].lower() == 'true' and (
                profile != 'mission' or values['imu_sidecar'].lower() == 'true'):
            raise RuntimeError('bulk_sidecarはmission専用でimu_sidecarと同時指定できません')
        # 時刻確認後の実行段階で命名し、launch定義の生成時刻を固定しない。
        if not values['bag_name']:
            values['bag_name'] = 'rosbag2_'+datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
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
            '--polling-interval', values['polling_interval_ms'],
            '--max-bag-size', values['max_bag_size_bytes'],
            '--max-cache-size', values['max_cache_messages'],
            '--writer-backend', values['writer_backend'],
            '--writer-queue-bytes', values['writer_queue_bytes'],
            '--writer-queue-items', values['writer_queue_items'],
            '--mcap-preset-profile', values['mcap_preset_profile'],
            '--qos-profile-overrides-path', values['qos_profile_overrides_path'],
            '--dds-profile', LaunchConfiguration('dds_profile').perform(context),
            *(['--dds-receive-diagnostics'] if
              LaunchConfiguration('dds_receive_diagnostics').perform(context).lower() == 'true' else []),
            *(['--imu-sidecar'] if values['imu_sidecar'].lower() == 'true' else []),
            *(['--bulk-sidecar'] if values['bulk_sidecar'].lower() == 'true' else []),
        ], output='screen', sigterm_timeout='120', sigkill_timeout='120')]
    # 既知の開発コンテナmountを優先し、Jetsonホストでは従来のhome配下へ保存する。
    workspace = (Path('/workspaces/patasmonkey_ws') if Path('/workspaces/patasmonkey_ws/src').is_dir()
                 else Path.home()/'patasmonkey_ws')
    external = (Path('/workspaces/ros2_ws') if Path('/workspaces/ros2_ws/src').is_dir()
                else Path.home()/'ros2_ws')
    root = workspace / 'bags'
    if profile == 'debug':
        root = root / 'debug'
    return LaunchDescription(launch_transport_actions() + [
        DeclareLaunchArgument('bag_directory', default_value=str(root)),
        DeclareLaunchArgument('bag_name', default_value='',
                              description='空なら録画開始時のシステム時刻で自動命名'),
        DeclareLaunchArgument('workspace', default_value=str(workspace)),
        DeclareLaunchArgument('external_workspace', default_value=str(external)),
        DeclareLaunchArgument('storage', default_value='mcap'),
        DeclareLaunchArgument('writer_backend', default_value='standard',
                              description='standardまたはasync。asyncも単一bagへ保存する'),
        DeclareLaunchArgument('writer_queue_bytes', default_value=str(64*1024*1024),
                              description='async専用FIFOのCDR payload上限[byte]'),
        DeclareLaunchArgument('writer_queue_items', default_value='8192',
                              description='async専用FIFOのitem上限。byte上限と同時に適用'),
        DeclareLaunchArgument('mcap_preset_profile', default_value='default',
                              description='default/none/zstd_fast_crc。Foxy対応MCAP overlayが必要'),
        DeclareLaunchArgument('max_cache_messages', default_value='0',
                              description='Foxy書き込みcache[message件数]。MCAPは0のみ許可'),
        DeclareLaunchArgument('imu_sidecar', default_value='false',
                              description='IMU専用bagを追加し、両bagの保存完了を確認する'),
        DeclareLaunchArgument('bulk_sidecar', default_value='false',
                              description='比較用: 画像・点群を排他的に別bagへ保存する'),
        DeclareLaunchArgument('max_bag_size_bytes', default_value='1000000000',
                              description='bag分割サイズ[byte]。0で分割無効、既定1 GB'),
        DeclareLaunchArgument('polling_interval_ms', default_value='100',
                              description='未発行topicの探索間隔[ms]。topicの取得周期ではない'),
        DeclareLaunchArgument('qos_profile_overrides_path', default_value='',
                              description='比較試験用の購読QoS YAML。空ならpackage標準設定'),
        DeclareLaunchArgument('mission_bag', default_value='',
                              description='debug bagと対応するmission bagの実パス（任意）'),
        DeclareLaunchArgument(
            'status_file',
            default_value=str(Path.home()/'.ros'/'pm_robot_manager'/
                              (profile+'_bag_status.json')),
            description='systemd ExecStopと共有するbag停止状態ファイル'),
        OpaqueFunction(function=start),
    ])
