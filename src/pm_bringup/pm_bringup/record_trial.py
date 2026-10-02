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
import yaml
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


class ParameterSnapshots(Node):
    """未発見/未応答nodeは失敗を明記し、次の低頻度巡回で再取得する。"""
    def __init__(self, output, profile):
        super().__init__(profile+'_bag_metadata')
        self.declare_parameter('output_directory',str(output))
        self.output = Path(output)
        self.parameters = {}
        self.errors = {}
        self.core_manifest = None

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
        self.started_at_unix_sec = None
        self._stop_requested = threading.Event()
        self._signal_lock = threading.Lock()
        self._sigint_sent = False
        prefix = '/pm/robot_manager/{}_bag'.format(profile)
        self._status_publisher = self.create_publisher(String, prefix+'/status', 10)
        self.create_service(Trigger, prefix+'/request_stop', self._stop_request)

    def set_child(self, child):
        self.child = child

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
            if (not self._sigint_sent and self.child is not None
                    and self.child.poll() is None):
                self._sigint_sent = True
                self.child.send_signal(signal.SIGINT)

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
    parser.add_argument('--mission-bag',default='')
    parser.add_argument(
        '--status-file',
        default=str(Path.home()/'.ros'/'pm_robot_manager'/'bag_status.json'))
    args = parser.parse_args(argv)
    output = Path(args.output).expanduser().absolute()
    if output.exists():
        raise RuntimeError('既存bagは上書きしません: '+str(output))
    share = Path(get_package_share_directory('pm_bringup'))
    profiles = yaml.safe_load((share/'config/bag_profiles.yaml').read_text())
    topics = profiles[args.profile]
    output.parent.mkdir(parents=True,exist_ok=True)
    command = ['ros2','bag','record','-o',str(output),'-s',args.storage,
               '--max-bag-size','1000000000', '--qos-profile-overrides-path',
               str(share/'config/bag_qos.yaml'), *topics]
    child = None
    control = None
    def request_stop(signum, frame):
        if control is not None:
            control.request_stop(publish=False)
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
        child = subprocess.Popen(command,start_new_session=True)
        control.set_child(child)
        control.started_at_unix_sec = time.time()
        control.publish_status('RECORDING')
        deadline = time.monotonic()+30.0
        while (not output.is_dir() and child.poll() is None
               and not stop_requested.is_set() and time.monotonic()<deadline):
            rclpy.spin_once(control,timeout_sec=.1)
            time.sleep(.1)
        if not output.is_dir():
            raise RuntimeError('recorderが出力directoryを作成できませんでした')
        provenance = output/'provenance'
        provenance.mkdir()
        manifest = {'profile':args.profile,'started_at_utc':utc_now(), 'command':command,
                    'topics':topics,'hostname':socket.gethostname(),'platform':platform.platform(),
                    'environment':{k:os.environ.get(k) for k in (
                        'ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','AMENT_PREFIX_PATH')},
                    'related_mission_bag':args.mission_bag}
        atomic_json(provenance/'recording.json',manifest)
        shutil.copy2(share/'config/bag_profiles.yaml',provenance/'bag_profiles.yaml')
        shutil.copy2(share/'config/bag_qos.yaml',provenance/'bag_qos.yaml')
        if args.profile == 'mission':
            node = ParameterSnapshots(provenance,args.profile)
            atomic_json(provenance/'versions.json',snapshot_versions(
                provenance/'git',args.workspace,args.external_workspace))
            atomic_json(provenance/'installed_packages.json',installed_inventory(provenance/'installed_sources'))
        next_capture = time.monotonic()
        next_status = time.monotonic()
        copied_configs = False
        while child.poll() is None and not stop_requested.is_set():
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
    finally:
        control.publish_status('STOPPING')
        if child is not None and child.poll() is None:
            control.request_stop()
            # 正常停止ではSIGKILLやSSH session切断を使わない。metadata確定まで待つ。
            child.wait()
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
                        and child is not None and child.returncode in (0,2))
            if not verified and not completion_error:
                completion_error = 'metadata.yamlまたはros2 bag infoの検証に失敗しました'
            atomic_json(output/'completion.json',{
                'stopped_at_utc':utc_now(),'recorder_returncode':child.returncode if child else None,
                'invocation_id':invocation_id,
                'metadata_exists':complete,
                'bag_info_returncode':info.returncode if info is not None else None,
                'verified':verified,'error':completion_error,
                'metadata_snapshot_error':snapshot_error,
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
