"""独立したrosbag recorderと、mission再現情報の低頻度snapshotを管理する。

rosbagが出力directoryを作る前に同じdirectoryを作らない。SIGINTは今回のchildだけへ
渡し、recorderの終了・metadata確定まで待つ。センサや車両制御は一切起動しない。
"""
import argparse
import collections
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import socket
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters, GetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import yaml
from pm_bringup.bag_integrity import verify_message_counts
from pm_bringup.dds_transport import transport_environment
from pm_bringup.recording_metadata import (
    atomic_json, copy_configuration, installed_inventory, redact, snapshot_versions, utc_now,
)


def parameter_value(value):
    """FoxyのParameterValueをJSONへ変換する。配列の型と単位はparameter名に従う。"""
    fields = {1:'bool_value', 2:'integer_value', 3:'double_value', 4:'string_value',
              5:'byte_array_value', 6:'bool_array_value', 7:'integer_array_value',
              8:'double_array_value', 9:'string_array_value'}
    if value.type not in fields:
        return None
    result = getattr(value,fields[value.type])
    return list(result) if value.type >= 5 else result


BULK_TOPICS = frozenset({
    '/oak/stereo/left/image_raw', '/oak/stereo/right/image_raw', '/oak/color/image_raw',
    '/oak/depth/image_raw', '/oak/stereo/confidence/image_raw', '/oak/stereo/disparity/image_raw',
    '/ov_msckf/points_msckf', '/ov_msckf/points_slam',
})


def partition_bulk_topics(topics):
    """画像・点群だけを別writerへ移し、topic集合を重複なく保存する。

    推定・TF・診断・IMUはmainへ残す。新しい未知topicはmainに残し、暗黙に
    記録対象から落とさない。これは任意の分離モードで、通常の単一bagを変えない。
    """
    return ([topic for topic in topics if topic not in BULK_TOPICS],
            [topic for topic in topics if topic in BULK_TOPICS])


def recorder_environment(preset, environment):
    """MCAP設定は所有childに限定する。defaultは実験presetの恒久的な継承を避ける。"""
    if preset not in ('default', 'none', 'zstd_fast_crc'):
        raise ValueError('未知のMCAP presetです')
    result = dict(environment)
    result.pop('PM_MCAP_PRESET_PROFILE', None)
    if preset != 'default':
        result['PM_MCAP_PRESET_PROFILE'] = preset
    return result


def verify_recorder(output, child):
    """停止済みの専用bagを検査する。終了前・metadata欠如は成功扱いしない。"""
    result = {'output': str(output), 'recorder_returncode': child.poll(),
              'metadata_exists': (output/'metadata.yaml').is_file(),
              'bag_info_returncode': None, 'verified': False, 'error': ''}
    if child.poll() is None:
        result['error'] = 'recorderがまだ終了していません'
        return result
    try:
        info = subprocess.run(['ros2', 'bag', 'info', str(output)],
                              capture_output=True, text=True, timeout=30)
        result['bag_info_returncode'] = info.returncode
        if output.is_dir():
            (output/'bag_info.txt').write_text(info.stdout+info.stderr, encoding='utf-8')
        result['verified'] = (result['metadata_exists'] and info.returncode == 0
                              and child.returncode in (0, 2))
        if result['verified']:
            result['content_integrity'] = verify_message_counts(output)
            result['verified'] = result['content_integrity']['verified']
            if not result['verified']:
                result['error'] = result['content_integrity']['error']
    except (OSError, subprocess.TimeoutExpired) as error:
        result['error'] = str(error)
    if not result['verified'] and not result['error']:
        result['error'] = '専用IMU bagの保存確認に失敗しました'
    return result


def select_qos_path(share, profile, override):
    """明示指定を優先し、missionだけ実機確認済みの有界履歴へ切り替える。"""
    if override:
        return Path(override).expanduser().absolute()
    return share/'config'/('bag_qos_mission_buffered.yaml'
                            if profile == 'mission' else 'bag_qos.yaml')


class ParameterSnapshots(Node):
    """未発見/未応答nodeは失敗を明記し、次の低頻度巡回で再取得する。"""
    def __init__(self, output, profile):
        super().__init__(profile+'_bag_metadata')
        self.declare_parameter('output_directory',str(output))
        self.output = Path(output)
        self.parameters = {}
        self.errors = {}
        self.core_manifest = None
        # カメラ側で読むEEPROMと設定をtopicに加え、閲覧用JSONとしてbagにも同梱する。
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.create_subscription(String, '/oak/stereo/recording_snapshot',
                                 self.capture_oak_snapshot,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def capture_oak_snapshot(self, message):
        """カメラの校正・設定・SDK情報を保存する。未取得の校正を推測で補わない。"""
        try:
            data = json.loads(message.data)
            if data.get('schema_version') != 1 or not data.get('mx_id'):
                raise ValueError('unsupported OAK snapshot schema/identity')
            atomic_json(self.output/'oak_stereo_snapshot.json', data)
            self.errors.pop('oak_stereo_snapshot', None)
        except (ValueError, TypeError) as error:
            self.errors['oak_stereo_snapshot'] = str(error)

    def capture(self, refresh=False):
        """snapshotは最大10秒。旧値を残したnodeには取得時刻を付け、完全取得を偽らない。"""
        deadline = time.monotonic()+10.0
        nodes = [namespace.rstrip('/')+'/'+name for name,namespace in self.get_node_names_and_namespaces()]
        duplicates = [name for name,count in collections.Counter(nodes).items() if count>1]
        for name in sorted(set(nodes)):
            if time.monotonic() >= deadline:
                break
            if name in duplicates:
                self.errors[name] = 'duplicate node name: parameter service owner is ambiguous'
                continue
            if name in self.parameters and not refresh:
                continue
            clients = []
            try:
                listing = self.create_client(ListParameters,name+'/list_parameters')
                getter = self.create_client(GetParameters,name+'/get_parameters')
                clients = [listing,getter]
                if not listing.service_is_ready() or not getter.service_is_ready():
                    self.errors[name] = 'parameter service not ready'
                    continue
                future = listing.call_async(ListParameters.Request())
                rclpy.spin_until_future_complete(self,future,timeout_sec=1.0)
                if not future.done() or future.result() is None:
                    self.errors[name] = 'list_parameters timeout'
                    continue
                names = future.result().result.names
                request = GetParameters.Request()
                request.names = names
                future = getter.call_async(request)
                rclpy.spin_until_future_complete(self,future,timeout_sec=1.0)
                if not future.done() or future.result() is None:
                    self.errors[name] = 'get_parameters timeout'
                    continue
                values = redact(dict(zip(names,(parameter_value(v) for v in future.result().values))))
                self.parameters[name] = {'captured_at_utc':utc_now(),'values':values}
                self.errors.pop(name,None)
                if name == '/pm_core_manifest':
                    self.core_manifest = json.loads(values['manifest_json'])
            except (KeyError, ValueError, RuntimeError) as error:
                self.errors[name] = str(error)
            finally:
                for client in clients:
                    self.destroy_client(client)
        atomic_json(self.output/'effective_parameters.json', {
            'updated_at_utc':utc_now(), 'graph_nodes':nodes,
            'snapshots':self.parameters,'errors':self.errors,
            'not_captured_nodes':sorted(set(nodes)-set(self.parameters)),
        })
        if self.core_manifest:
            atomic_json(self.output/'core_manifest.json',self.core_manifest)


class BagControl(Node):
    """managerからbag停止を受け、保存確認を伴う終了状態をpublishする。"""
    def __init__(self, profile, output, status_file, invocation_id):
        super().__init__(profile+'_bag_control')
        self.profile = profile
        self.output = Path(output)
        self.status_file = Path(status_file)
        self.invocation_id = invocation_id
        self.status_file_error = ''
        # systemd ExecStopはROS statusの再配信を受けられないため、同じ起動IDで
        # status fileを照合できるようにする。初回書込みに失敗した場合は録画を開始しない。
        self.status_file.parent.mkdir(parents=True, exist_ok=True)
        self.child = None
        self.children = []
        self.started_at_unix_sec = None
        self._stop_requested = threading.Event()
        # Pythonのsignal handlerが同じthreadのchild登録へ割り込む場合にも再入可能にする。
        self._signal_lock = threading.RLock()
        self._sigint_sent = False
        prefix = '/pm/robot_manager/{}_bag'.format(profile)
        self._status_publisher = self.create_publisher(String, prefix+'/status', 10)
        self.create_service(Trigger, prefix+'/request_stop', self._stop_request)

    def set_child(self, child):
        self.child = child
        self.add_child(child)

    def add_child(self, child):
        """所有するrecorderだけを登録する。起動中に停止要求が来ても新childを止める。"""
        with self._signal_lock:
            self.children.append(child)
            if self._stop_requested.is_set() and child.poll() is None:
                try:
                    child.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass  # poll直後に自然終了したchildはmainのwaitで回収する。

    def publish_status(self, state, error='', verified=None):
        """状態とJetson上の出力先をROS経由でmanagerへ渡す。"""
        payload = {
            'profile': self.profile,
            'invocation_id': self.invocation_id,
            'state': state,
            'output': str(self.output),
            'started_at_unix_sec': self.started_at_unix_sec,
            'stamp_unix_sec': time.time(),
            'error': error,
            'verified': verified,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(message)
        try:
            atomic_json(self.status_file, payload)
        except OSError as error:
            self.status_file_error = str(error)
            self.get_logger().error(
                'systemd停止確認用status fileを書き込めません: {}'.format(error))

    def request_stop(self, publish=True):
        """rosbag子processへSIGINTを一度だけ送り、終了確認はmainで行う。"""
        self._stop_requested.set()
        if publish:
            self.publish_status('STOPPING')
        with self._signal_lock:
            if not self._sigint_sent:
                self._sigint_sent = True
                # 全recorderへ先にSIGINTを送り、その後mainで各終了を待つ。
                # 一方のclose待ちの間に他方を記録し続けない。
                for child in self.children:
                    if child.poll() is None:
                        try:
                            child.send_signal(signal.SIGINT)
                        except ProcessLookupError:
                            pass  # 一方の終了競合で他方への停止signalを省略しない。

    def _stop_request(self, _request, response):
        self.request_stop()
        response.success = True
        response.message = 'bag停止を受け付けました。metadataとros2 bag infoの確認後に終了します'
        return response


def main(argv=None):
    """recorderだけを子sessionで起動し、missionは実行条件も同梱する。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',choices=['mission','debug'],required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--workspace',required=True)
    parser.add_argument('--external-workspace',required=True)
    parser.add_argument('--storage',default='mcap')
    parser.add_argument('--metadata-capture-timing', choices=['during_recording', 'after_recording'],
                        default='after_recording',
                        help='mission再現情報の収集時期。afterは記録終了時のsnapshotで開始時の保証ではない')
    parser.add_argument('--writer-backend', choices=['standard', 'async'], default='standard',
                        help='単一bag保存経路。asyncは有界FIFOでstorage writeを別threadへ渡す')
    parser.add_argument('--writer-queue-bytes', type=int, default=64*1024*1024,
                        help='async専用FIFOのCDR payload上限[byte]。書込み中も含む')
    parser.add_argument('--writer-queue-items', type=int, default=8192,
                        help='async専用FIFOのitem上限。messageとtopic登録/削除を含む')
    parser.add_argument('--mcap-preset-profile', choices=['default', 'none', 'zstd_fast_crc'],
                        default='default', help='Foxy overlayのMCAP可逆圧縮。defaultは従来動作')
    parser.add_argument('--max-bag-size', type=int, default=2000000000,
                        help='bag分割サイズ[byte]。0で分割無効、既定2 GB（10進）')
    parser.add_argument('--polling-interval', type=int, default=100,
                        help='未発行topicを再探索する間隔[ms]。取得周期ではない')
    parser.add_argument('--max-cache-size', type=int, default=0,
                        help='Foxy書き込みcache容量[message件数]。MCAPでは0のみ許可')
    parser.add_argument('--qos-profile-overrides-path', default='',
                        help='比較試験用の購読QoS YAML。空ならpackageの標準設定')
    parser.add_argument('--dds-profile', default='',
                        help='空なら既定UDPv4＋SHM 2 MiBのXMLを使う')
    parser.add_argument('--dds-receive-diagnostics', action='store_true',
                        help='明示した受信診断hookの環境を継承する。既定OFF')
    parser.add_argument('--mission-bag',default='')
    parser.add_argument('--imu-sidecar', action='store_true',
                        help='missionと同時にIMU専用bagを追加記録し、両方の保存完了を待つ')
    parser.add_argument('--bulk-sidecar', action='store_true',
                        help='比較用: 画像・点群を別bagへ排他的に分離する。既定OFF')
    parser.add_argument(
        '--status-file',
        default=str(Path.home()/'.ros'/'pm_robot_manager'/'bag_status.json'))
    args = parser.parse_args(argv)
    if args.polling_interval <= 0:
        parser.error('--polling-intervalは正のミリ秒で指定してください')
    if args.writer_queue_bytes <= 0:
        parser.error('--writer-queue-bytesは正のbyte数で指定してください')
    if args.writer_queue_items <= 0:
        parser.error('--writer-queue-itemsは正のitem数で指定してください')
    if args.mcap_preset_profile != 'default' and args.storage != 'mcap':
        parser.error('--mcap-preset-profileはMCAP専用です')
    if args.max_bag_size < 0:
        parser.error('--max-bag-sizeは0以上のbyte数で指定してください')
    if args.max_cache_size < 0:
        parser.error('--max-cache-sizeは0以上のmessage件数で指定してください')
    if args.storage == 'mcap' and args.max_cache_size != 0:
        parser.error('Foxy/MCAPの非zero cacheは保存欠落が確認されたため使用できません')
    if args.writer_backend == 'async' and args.max_cache_size != 0:
        parser.error('async writerは標準cacheを併用せず、--max-cache-size 0を指定してください')
    if args.imu_sidecar and args.profile != 'mission':
        parser.error('--imu-sidecarはmission専用です')
    if args.bulk_sidecar and (args.profile != 'mission' or args.imu_sidecar):
        parser.error('--bulk-sidecarはmission専用で、--imu-sidecarと同時指定できません')
    output = Path(args.output).expanduser().absolute()
    if output.exists():
        raise RuntimeError('既存bagは上書きしません: '+str(output))
    share = Path(get_package_share_directory('pm_bringup'))
    child_environment = recorder_environment(args.mcap_preset_profile, os.environ)
    child_environment = transport_environment(args.dds_profile, child_environment,
                                              args.dds_receive_diagnostics)
    preset_support = None
    if args.mcap_preset_profile != 'default':
        # 古いFoxy pluginはこの環境変数を無視する。対応overlayのmarkerが
        # ない場合は起動前に拒否し、実際には非圧縮だった比較を作らない。
        preset_support = Path(get_package_share_directory('rosbag2_storage_mcap'))/'pm_foxy_preset_support.yaml'
        if (not preset_support.is_file() or args.mcap_preset_profile not in
                yaml.safe_load(preset_support.read_text()).get('profiles', [])):
            raise RuntimeError('MCAP preset対応overlayをbuild/sourceしてください')
    # 第二recorderの停止・検証経路は実績のあるIMU専用bagと共有する。
    # 既存の第二recorder管理経路を再利用し、imu_*変数はbulk時には画像・点群bagを指す。
    auxiliary = args.imu_sidecar or args.bulk_sidecar
    imu_output = Path(str(output)+('_bulk' if args.bulk_sidecar else '_imu'))
    imu_qos = share/'config'/('bag_qos_bulk_buffered.yaml' if args.bulk_sidecar
                             else 'bag_qos_imu_buffered.yaml')
    if auxiliary and (imu_output.exists() or not imu_qos.is_file()):
        raise RuntimeError('専用bagの既存出力またはQoS設定を確認してください: '+str(imu_output))
    profiles = yaml.safe_load((share/'config/bag_profiles.yaml').read_text())
    topics = profiles[args.profile]
    auxiliary_topics = ['/oak/imu/data', '/wit/imu']
    if args.bulk_sidecar:
        topics, auxiliary_topics = partition_bulk_topics(topics)
        if not topics or not auxiliary_topics:
            raise RuntimeError('分離するtopic集合が空です')
    # 探索周期と購読QoSを外部から指定し、同じrecorderで一要因ずつ比較する。
    # 新topicの開始待ちには探索周期分の遅延が増えるが、既存topicのレートは変えない。
    qos_path = select_qos_path(share, args.profile, args.qos_profile_overrides_path)
    if not qos_path.is_file():
        raise RuntimeError('購読QoS設定が存在しません: '+str(qos_path))
    output.parent.mkdir(parents=True,exist_ok=True)
    command = ['ros2','bag','record','-o',str(output),'-s',args.storage,
               '--max-cache-size', str(args.max_cache_size),
               '--max-bag-size',str(args.max_bag_size), '--polling-interval',str(args.polling_interval),
               '--qos-profile-overrides-path',str(qos_path), *topics]
    if args.writer_backend == 'async':
        # 外部ROS依存を書き換えず、標準transport＋非同期writerを選ぶ。
        # 実binaryを直接所有し、従来と同じSIGINT・停止待ち・件数検証を適用する。
        executable = Path(get_package_prefix('pm_recording'))/'lib/pm_recording/async_recorder'
        if not executable.is_file():
            raise RuntimeError('pm_recordingをbuild/sourceしてください: '+str(executable))
        # byteとitemの両上限を明示する。画像と小さいIMUの構成比で先に達する
        # 上限が変わるため、片方だけ増やして長い停滞を吸収したと見なさない。
        command = [str(executable), '--queue-bytes', str(args.writer_queue_bytes),
                   '--queue-messages', str(args.writer_queue_items), *command[3:]]
    child = None
    imu_child = None
    imu_log = None
    imu_result = None
    control = None
    def request_stop(signum, frame):
        if control is not None:
            control.request_stop(publish=False)
    # 直接CLI起動でもmetadata/control participantとrecorderを同じXMLに揃える。
    # このprocessだけを変更し、呼出元shellには書き戻さない。
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = child_environment['FASTRTPS_DEFAULT_PROFILES_FILE']
    rclpy.init(args=[])
    invocation_id = os.environ.get('INVOCATION_ID', '')
    try:
        control = BagControl(args.profile, output, args.status_file, invocation_id)
    except Exception:
        if rclpy.ok():
            rclpy.shutdown()
        raise
    stop_requested = control._stop_requested
    control.publish_status('STARTING')
    if control.status_file_error:
        control.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(
            '保存確認用status fileを作成できないため録画を開始しません: '+
            control.status_file_error)
    previous = {s:signal.signal(s,request_stop) for s in (signal.SIGINT,signal.SIGTERM)}
    node = None
    info = None
    verified = False
    completion_error = ''
    snapshot_error = ''
    try:
        child = subprocess.Popen(command, env=child_environment, start_new_session=True)
        control.set_child(child)
        if auxiliary:
            # IMU modeは重複保存、bulk modeは画像・点群をmainから除外する。
            # 両modeで同じ所有child登録・SIGINT停止・実件数検証を使う。
            imu_command = ['ros2', 'bag', 'record', '-o', str(imu_output),
                           '-s', args.storage, '--max-cache-size', '0',
                           '--max-bag-size', str(args.max_bag_size),
                           '--polling-interval', str(args.polling_interval),
                           '--qos-profile-overrides-path', str(imu_qos),
                           *auxiliary_topics]
            imu_log = Path(str(imu_output)+'_record.log').open('w', encoding='utf-8')
            imu_child = subprocess.Popen(imu_command, stdout=imu_log,
                                         stderr=subprocess.STDOUT, env=child_environment,
                                         start_new_session=True)
            control.add_child(imu_child)
        control.started_at_unix_sec = time.time()
        control.publish_status('RECORDING')
        deadline = time.monotonic()+30.0
        while (not output.is_dir() and child.poll() is None
               and not stop_requested.is_set() and time.monotonic()<deadline):
            rclpy.spin_once(control,timeout_sec=.1)
            time.sleep(.1)
        if not output.is_dir() and stop_requested.is_set():
            # 起動直後の停止はdirectory未作成でも異常ではない。finallyで
            # 子process終了を待ち、実データなしの取消証明を別ファイルに残す。
            return
        if not output.is_dir():
            raise RuntimeError('recorderが出力directoryを作成できませんでした')
        provenance = output/'provenance'
        provenance.mkdir()
        manifest = {'profile':args.profile,'started_at_utc':utc_now(), 'command':command,
                    'metadata_capture_timing': args.metadata_capture_timing,
                    'metadata_scope': ('recording_and_end' if args.metadata_capture_timing == 'during_recording'
                                       else 'post_recording_only_not_start_snapshot'),
                    'topics':topics,'hostname':socket.gethostname(),'platform':platform.platform(),
                    'mcap_preset_profile': args.mcap_preset_profile,
                    'writer_backend': args.writer_backend,
                    'writer_queue_bytes': args.writer_queue_bytes,
                    'writer_queue_items': args.writer_queue_items,
                    'environment':{k:child_environment.get(k) for k in (
                        'ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','AMENT_PREFIX_PATH',
                        'PM_MCAP_PRESET_PROFILE', 'FASTRTPS_DEFAULT_PROFILES_FILE')},
                    'related_mission_bag':args.mission_bag}
        atomic_json(provenance/'recording.json',manifest)
        shutil.copy2(child_environment['FASTRTPS_DEFAULT_PROFILES_FILE'],
                     provenance/'dds_transport.xml')
        if preset_support is not None:
            shutil.copy2(preset_support, provenance/'mcap_preset_support.yaml')
        if auxiliary:
            atomic_json(provenance/('bulk_sidecar.json' if args.bulk_sidecar else 'imu_sidecar.json'), {
                'output': str(imu_output), 'command': imu_command,
                'topics': auxiliary_topics,
                'topics_duplicated_in_main_bag': not args.bulk_sidecar})
            shutil.copy2(imu_qos, provenance/('bulk_sidecar_qos.yaml' if args.bulk_sidecar
                                            else 'imu_sidecar_qos.yaml'))
            if args.bulk_sidecar:
                # mainだけでは全missionが揃わない。両bagの担当と関連を明示する。
                atomic_json(provenance/'topic_partition.json', {
                    'main': str(output), 'bulk': str(imu_output),
                    'main_topics': topics, 'bulk_topics': auxiliary_topics,
                    'combined_topics': profiles[args.profile],
                    'requires_both_bags': True})
        shutil.copy2(share/'config/bag_profiles.yaml',provenance/'bag_profiles.yaml')
        # 標準設定ではなく今回実際に使用した設定を保存し、A/B条件の取り違えを防ぐ。
        shutil.copy2(qos_path,provenance/'bag_qos.yaml')
        if args.profile == 'mission' and args.metadata_capture_timing == 'during_recording':
            node = ParameterSnapshots(provenance,args.profile)
            atomic_json(provenance/'versions.json',snapshot_versions(
                provenance/'git',args.workspace,args.external_workspace))
            atomic_json(provenance/'installed_packages.json',installed_inventory(provenance/'installed_sources'))
        next_capture = time.monotonic()
        next_status = time.monotonic()
        copied_configs = False
        while child.poll() is None and not stop_requested.is_set():
            if imu_child is not None and imu_child.poll() is not None:
                raise RuntimeError('第二recorderが録画中に終了しました')
            rclpy.spin_once(control,timeout_sec=.1)
            if node is not None:
                rclpy.spin_once(node,timeout_sec=.2)
                if time.monotonic() >= next_capture:
                    node.capture()
                    if node.core_manifest and not copied_configs:
                        atomic_json(provenance/'configuration_index.json',copy_configuration(
                            node.core_manifest.get('configuration_paths',[]),provenance/'configuration'))
                        copied_configs = True
                    next_capture = time.monotonic()+10.0
            if time.monotonic() >= next_status:
                control.publish_status('RECORDING')
                next_status = time.monotonic()+1.0
        if not stop_requested.is_set() and child.poll() is not None:
            raise RuntimeError('mission/debug recorderが停止要求なしに終了しました')
    except Exception as error:
        completion_error = str(error)
        raise
    finally:
        control.publish_status('STOPPING')
        control.request_stop()
        # 正常停止ではSIGKILLやSSH切断を使わず、全childのmetadata確定まで待つ。
        for recorder in control.children:
            recorder.wait()
        if imu_log is not None:
            imu_log.close()
        if imu_child is not None:
            imu_result = verify_recorder(imu_output, imu_child)
            if imu_output.is_dir():
                atomic_json(imu_output/'completion.json', imu_result)
        if (args.profile == 'mission' and args.metadata_capture_timing == 'after_recording'
                and output.is_dir() and (output/'provenance').is_dir()):
            try:
                # writerが閉じてからGit・ソース収集・ROS問い合わせを開始する。
                # coreは継続稼働が必要。記録開始時情報と偽らず終了時のscopeを明記する。
                phase_start = time.monotonic()
                phase_utc = utc_now()
                provenance = output/'provenance'
                node = ParameterSnapshots(provenance, args.profile)
                atomic_json(provenance/'versions.json', snapshot_versions(
                    provenance/'git', args.workspace, args.external_workspace))
                atomic_json(provenance/'installed_packages.json', installed_inventory(provenance/'installed_sources'))
                # 後から参加したmetadata nodeのgraphとtransient-local校正情報を取得する。
                # センサやモータへの設定・制御commandは送らない。
                until = time.monotonic()+2.0
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.05)
                atomic_json(provenance/'metadata_deferred_phase.json', {
                    'started_at_utc': phase_utc, 'scope': 'post_recording_only',
                    'pre_parameter_capture_duration_sec': time.monotonic()-phase_start,
                    'warning': '終了時のsnapshot。記録中の設定・ソース変更は自動検知しません'})
            except Exception as error:
                # メタデータの収集失敗でbag本体の検証・完了証明を省略しない。
                snapshot_error = str(error)
        if node is not None:
            try:
                node.capture(refresh=True)
                if not node.core_manifest:
                    atomic_json(node.output/'core_manifest_missing.json',{
                        'reason':'pm_core_manifest未取得。起動条件の完全保存はできていません'})
                else:
                    if not (node.output/'configuration_index.json').is_file():
                        atomic_json(node.output/'configuration_index.json',copy_configuration(
                            node.core_manifest.get('configuration_paths',[]),node.output/'configuration'))
                    runtime = Path(node.core_manifest['runtime_directory'])
                    # recorder停止時点の付帯ログ。coreが継続稼働中ならそれ以後の内容は含まない。
                    if runtime.is_dir():
                        shutil.copytree(runtime,node.output/'runtime_artifacts')
            except Exception as error:
                # Mission補助metadataの失敗でbag本体のclose・検証を飛ばさない。
                snapshot_error = str(error)
            finally:
                node.destroy_node()
        if output.is_dir():
            complete = (output/'metadata.yaml').is_file()
            try:
                info = subprocess.run(
                    ['ros2','bag','info',str(output)],capture_output=True,
                    text=True,timeout=30)
                (output/'bag_info.txt').write_text(
                    info.stdout+info.stderr,encoding='utf-8')
            except (OSError, subprocess.TimeoutExpired) as error:
                completion_error = 'ros2 bag infoを完了できません: {}'.format(error)
                (output/'bag_info.txt').write_text(str(error),encoding='utf-8')
            verified = (complete and info is not None and info.returncode == 0
                        and child is not None and child.returncode in (0,2)
                        and not completion_error)
            content_integrity = verify_message_counts(output) if verified else None
            if verified and args.writer_backend == 'async':
                # transportは内部例外をlogして戻る場合がある。FIFO flush証明の
                # 欠如・不一致をmetadata成功で打ち消さず、保存失敗として扱う。
                try:
                    async_state = yaml.safe_load((output/'async_writer.yaml').read_text())
                    verified = (async_state.get('verified') is True
                                and async_state.get('accepted') == async_state.get('written')
                                and async_state.get('written') == content_integrity['actual_total'])
                except (OSError, ValueError, AttributeError, KeyError, yaml.YAMLError):
                    verified = False
                if not verified:
                    completion_error = '非同期writerのflush証明または実保存件数が一致しません'
            if content_integrity is not None and not content_integrity['verified']:
                verified = False
                completion_error = content_integrity['error']
            # GUI/systemdには従来と同じmain completionを返すが、成功条件は両bag。
            # 専用bag失敗時にmainだけを保存済みと表示することを防ぐ。
            if auxiliary:
                verified = verified and imu_result is not None and imu_result['verified']
                if imu_result is not None and not imu_result['verified']:
                    completion_error = imu_result['error']
            if not verified and not completion_error:
                completion_error = 'metadata.yamlまたはros2 bag infoの検証に失敗しました'
            atomic_json(output/'completion.json',{
                'stopped_at_utc':utc_now(),'recorder_returncode':child.returncode if child else None,
                'invocation_id':invocation_id,
                'metadata_exists':complete,
                'bag_info_returncode':info.returncode if info is not None else None,
                'verified':verified,'error':completion_error,
                'metadata_snapshot_error':snapshot_error,
                'content_integrity': content_integrity,
                'imu_sidecar': imu_result if args.imu_sidecar else None,
                'bulk_sidecar': imu_result if args.bulk_sidecar else None,
            })
        elif (not auxiliary and stop_requested.is_set() and child is not None
              and child.returncode in (0, 2, -signal.SIGINT, 130)):
            # 保存済みbagとは区別する。SIGINT終了とdirectory不在の両方が
            # 確認できた場合だけ、保存すべきデータがない取消として扱う。
            verified = True
            atomic_json(Path(str(output)+'.cancelled.json'), {
                'invocation_id': invocation_id, 'cancelled_before_output': True,
                'recorder_returncode': child.returncode, 'verified': True,
            })
        elif not completion_error:
            completion_error = 'bag出力directoryがなく、保存完了を確認できません'
        control.publish_status(
            'STOPPED' if verified else 'ERROR',
            error=completion_error, verified=verified)
        for sig, handler in previous.items():
            signal.signal(sig,handler)
        control.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not verified:
        raise RuntimeError('bag終了状態を確認してください: '+str(output))


if __name__ == '__main__':
    main()
