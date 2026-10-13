"""coreと独立recorderで共用するDDS通信設定。親shellの環境は変更しない。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def default_profile():
    """インストール先からXMLを解決し、ホスト固定パスを避ける。"""
    return str(Path(get_package_share_directory('pm_bringup'))/'config/dds_transport_2m.xml')


def transport_environment(profile, environment, diagnostics=False):
    """子環境を作る。受信hookは明示ON時だけ継承し、他のpreloadは維持する。

    XMLの明示指定は比較/ロールバック用。RMWの種類は勝手に変更しない。
    """
    path = Path(profile or default_profile()).expanduser().resolve()
    if not path.is_file():
        raise ValueError('DDS XMLが存在しません: '+str(path))
    result = dict(environment)
    result['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(path)
    if not diagnostics:
        for key in ('PM_DDS_TRACE', 'PM_RCL_TRACE_PATH'):
            result.pop(key, None)
        libraries = result.get('LD_PRELOAD', '').replace(':', ' ').split()
        libraries = [item for item in libraries if Path(item).name != 'librcl_receive_trace.so']
        if libraries:
            result['LD_PRELOAD'] = ' '.join(libraries)
        else:
            result.pop('LD_PRELOAD', None)
    return result


def launch_transport_actions():
    """node生成前にlaunch contextへ適用する。sensorは起動しない。"""
    def configure(context):
        enabled = LaunchConfiguration('dds_receive_diagnostics').perform(context).lower()
        if enabled not in ('true', 'false'):
            raise ValueError('dds_receive_diagnosticsはtrue/falseで指定してください')
        updated = transport_environment(LaunchConfiguration('dds_profile').perform(context),
                                        context.environment, enabled == 'true')
        for key in ('PM_DDS_TRACE', 'PM_RCL_TRACE_PATH', 'LD_PRELOAD'):
            context.environment.pop(key, None)
        context.environment.update(updated)
        return []
    return [DeclareLaunchArgument('dds_profile', default_value=default_profile(),
                                 description='Fast DDS XML。既定UDPv4＋SHM 2 MiB/participant'),
            DeclareLaunchArgument('dds_receive_diagnostics', default_value='false',
                                 description='明示指定したLD_PRELOAD等の受信診断を継承する'),
            OpaqueFunction(function=configure)]
