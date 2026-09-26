"""ROS通信、テレメトリ、GUI所有rosbag processを管理するbackend。"""

import json
import math
import os
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Joy, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from ublox_msgs.msg import NavPVT
except ImportError:  # ublox driver未導入の開発環境でも他の画面を利用可能にする。
    NavPVT = None


def _quaternion_rpy(q):
    """ROS quaternionからroll/pitch/yaw [rad]を計算する。"""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


class RosBackend(Node):
    """ROS executor threadからデータを集約し、Qtへsnapshotを渡す。"""

    def __init__(self, config):
        super().__init__('pm_gui_backend')
        manager = config['manager']
        self.backend_mode = manager.get('backend', 'ros')
        self.status_topic = manager['status_topic']
        self.start_service_name = manager['start_service']
        self.stop_service_name = manager['stop_service']
        self.get_status_service_name = manager['get_status_service']
        self.stale_timeout = float(manager.get('stale_timeout_sec', 3.0))
        topics = config.get('topics', {})
        self.topic_timeout = float(topics.get('stale_timeout_sec', 2.0))
        self._lock = threading.RLock()
        self._state = 'UNKNOWN'
        self._error = ''
        self._unit = ''
        self._last_heartbeat = None
        self._last_response = ''
        self._service_names = {self.start_service_name, self.stop_service_name,
                               self.get_status_service_name}
        self._telemetry = {}
        self._camera_enabled = False
        self._camera_config = config.get('camera', {})
        self._camera_subscription = None
        self._camera_image = None
        self._camera_stamp = None
        self._camera_received_stamp = None
        self._camera_subscription_started = None
        self._camera_error = ''
        self._camera_receive_count = 0
        self._camera_converted_count = 0
        self._camera_conversion_error_count = 0
        self._recordings = {}
        self._recording_config = config.get('recording', {})
        self._recording_stop_pending = False

        self._status_subscription = self.create_subscription(
            String, self.status_topic, self._status_callback, 10)
        self._start_client = self.create_client(Trigger, self.start_service_name)
        self._stop_client = self.create_client(Trigger, self.stop_service_name)
        self._status_client = self.create_client(Trigger, self.get_status_service_name)
        self._telemetry_subscriptions = []
        self._add_subscription(Odometry, topics.get('odometry', '/odometry/global'),
                               self._odometry_callback)
        self._add_subscription(Twist, topics.get('cmd_vel', '/cmd_vel_joy'),
                               self._twist_callback)
        self._add_subscription(Joy, topics.get('joy', '/pm/joy'), self._joy_callback)
        self._add_subscription(NavSatFix, topics.get('gnss_fix', '/fix'),
                               self._fix_callback)
        if NavPVT is not None and topics.get('navpvt', '/navpvt'):
            self._add_subscription(NavPVT, topics.get('navpvt', '/navpvt'),
                                   self._navpvt_callback)
        self._telemetry_timer = self.create_timer(0.2, self._telemetry_tick)

        if self.backend_mode == 'mock':
            self._mock_state = 'STOPPED'
            self._mock_deadline = None
            self._mock_target = None
            self._mock_error_mode = False
            self.create_service(Trigger, self.start_service_name, self._mock_start)
            self.create_service(Trigger, self.stop_service_name, self._mock_stop)
            self.create_service(Trigger, self.get_status_service_name, self._mock_get_status)
            self.create_service(Trigger, '/pm/gui/mock_error', self._mock_error_request)
            self._mock_error_client = self.create_client(Trigger, '/pm/gui/mock_error')
            self.create_timer(0.25, self._mock_tick)
        else:
            self._status_probe_timer = self.create_timer(1.0, self._request_initial_status)

    def _add_subscription(self, msg_type, topic, callback):
        if topic:
            self._telemetry_subscriptions.append(
                self.create_subscription(msg_type, topic, callback, 10))

    def _store(self, key, value):
        with self._lock:
            self._telemetry[key] = (value, time.monotonic())

    def _odometry_callback(self, message):
        pose = message.pose.pose
        roll, pitch, yaw = _quaternion_rpy(pose.orientation)
        self._store('odometry', {
            'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'frame_id': message.header.frame_id,
            'child_frame_id': message.child_frame_id,
        })

    def _twist_callback(self, message):
        self._store('twist', {'linear_x': message.linear.x, 'angular_z': message.angular.z})

    def _joy_callback(self, message):
        self._store('joy', {'buttons': list(message.buttons), 'axes': list(message.axes)})

    def _fix_callback(self, message):
        self._store('fix', {
            'status': message.status.status, 'latitude': message.latitude,
            'longitude': message.longitude, 'altitude': message.altitude,
        })

    def _navpvt_callback(self, message):
        # u-blox carrier phase flags distinguish RTK FLOAT/FIX; NavSatFix status alone cannot.
        flags = int(message.flags)
        carrier = flags & int(message.FLAGS_CARRIER_PHASE_MASK)
        state = 'NO FIX'
        if (flags & int(message.FLAGS_GNSS_FIX_OK)) and int(message.fix_type) >= int(message.FIX_TYPE_3D):
            state = ('RTK FIX' if carrier == int(message.CARRIER_PHASE_FIXED)
                     else 'RTK FLOAT' if carrier == int(message.CARRIER_PHASE_FLOAT)
                     else 'GNSS')
        self._store('navpvt', {'gnss_state': state, 'fix_type': int(message.fix_type)})

    def _telemetry_tick(self):
        # camera subscriptionの生成・破棄はROS executor内で行い、Qt threadとの競合を避ける。
        with self._lock:
            enabled = self._camera_enabled
            subscription = self._camera_subscription
            if enabled and subscription is None:
                topic = self._camera_config.get('topic', '/oak/color/image_raw')
                # BEST_EFFORT subscriberはRELIABLE publisherとも互換で、古い画像の再送待ちを避ける。
                qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
                self._camera_subscription = self.create_subscription(
                    Image, topic, self._image_callback, qos)
                self._camera_subscription_started = time.monotonic()
            elif not enabled and subscription is not None:
                self.destroy_subscription(subscription)
                self._camera_subscription = None
                self._camera_subscription_started = None
                self._camera_image = None
                self._camera_stamp = None
                self._camera_error = ''
            stopping = self._recording_stop_pending
        if stopping:
            self._finish_core_stop_after_recordings()

    def _image_callback(self, message):
        with self._lock:
            self._camera_receive_count += 1
            receive_count = self._camera_receive_count
            self._camera_received_stamp = time.monotonic()
        if receive_count == 1 or receive_count % 25 == 0:
            self.get_logger().info(
                '画像callback受信: topic={} count={}'.format(
                    self._camera_config.get('topic', '/oak/color/image_raw'), receive_count))
        try:
            from cv_bridge import CvBridge
            import numpy as np
            bridge = getattr(self, '_bridge', None)
            if bridge is None:
                bridge = CvBridge()
                self._bridge = bridge
            # decodeはROS threadで行い、Qt timerは最新の小さな画像参照だけを描画する。
            frame = bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
            frame = np.ascontiguousarray(frame)
            with self._lock:
                self._camera_image = frame
                self._camera_stamp = time.monotonic()
                self._camera_converted_count += 1
                self._camera_error = ''
        except Exception as exc:
            with self._lock:
                self._camera_conversion_error_count += 1
                self._camera_error = str(exc)
                error_count = self._camera_conversion_error_count
            self.get_logger().error(
                'cv_bridge RGB変換失敗: topic={} receive_count={} error_count={} error={!r}'.format(
                    self._camera_config.get('topic', '/oak/color/image_raw'),
                    receive_count, error_count, str(exc)))

    def set_camera_enabled(self, enabled):
        with self._lock:
            self._camera_enabled = bool(enabled)

    def start_recording(self, profile):
        settings = self._recording_config
        profiles = settings.get('profiles', {})
        selected = profiles.get(profile)
        if not selected:
            return False, '記録topic profileが未設定です'
        topics = list(dict.fromkeys(selected.get('topics', [])))
        if not topics:
            return False, '記録topicが空です'
        with self._lock:
            existing = self._recordings.get(profile)
            if existing and existing['state'] in ('RECORDING', 'STOPPING'):
                return False, 'このprofileは記録中または停止処理中です'
        output_root = os.path.expanduser(settings.get('output_directory', '~/patasmonkey_ws/bags/gui'))
        os.makedirs(output_root, exist_ok=True)
        label = profile.lower()
        output = os.path.join(output_root, '{}_{}'.format(
            label, datetime.now().strftime('%Y%m%d_%H%M%S')))
        command = ['ros2', 'bag', 'record', '-s', settings.get('storage_id', 'mcap'),
                   '-o', output]
        compression_mode = settings.get('compression_mode', 'none')
        if compression_mode != 'none':
            command.extend(['--compression-mode', str(compression_mode),
                            '--compression-format', str(settings.get('compression_format', 'zstd'))])
        max_size = int(settings.get('max_bag_size_bytes', 0))
        if max_size > 0:
            command.extend(['--max-bag-size', str(max_size)])
        command.extend(topics)
        try:
            process = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL, start_new_session=True)
        except OSError as exc:
            return False, 'ros2 bagを起動できません: {}'.format(exc)
        with self._lock:
            self._recordings[profile] = {
                'process': process, 'output': output, 'started': time.monotonic(),
                'state': 'RECORDING', 'error': '',
            }
        return True, output

    def stop_recording(self, profile):
        with self._lock:
            item = self._recordings.get(profile)
            if not item or item['state'] != 'RECORDING':
                return False
            item['state'] = 'STOPPING'
        threading.Thread(target=self._stop_process, args=(profile,), daemon=True).start()
        return True

    def _stop_process(self, profile):
        with self._lock:
            item = self._recordings.get(profile)
        if not item:
            return
        process = item['process']
        try:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=float(self._recording_config.get('stop_timeout_sec', 12.0)))
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        except ProcessLookupError:
            pass
        with self._lock:
            if profile in self._recordings:
                self._recordings[profile]['state'] = 'STOPPED'

    def _finish_core_stop_after_recordings(self):
        with self._lock:
            if any(item['state'] in ('RECORDING', 'STOPPING') for item in self._recordings.values()):
                return
            self._recording_stop_pending = False
        self._request(self._stop_client)

    def request_start(self):
        return self._request(self._start_client)

    def request_stop(self):
        """GUI所有bagを閉じてからmanagerへCore STOPを依頼する。"""
        with self._lock:
            active = [name for name, item in self._recordings.items()
                      if item['state'] in ('RECORDING', 'STOPPING')]
            if active:
                self._recording_stop_pending = True
        if active:
            for name in active:
                self.stop_recording(name)
            return True
        return self._request(self._stop_client)

    def _request(self, client):
        if not client.service_is_ready():
            with self._lock:
                self._last_response = 'Robot Manager serviceが見つかりません'
            return False
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self._service_response)
        return True

    def _decode_status(self, payload):
        try:
            data = json.loads(payload)
            state = str(data.get('state', 'UNKNOWN')).upper()
            if state not in ('STOPPED', 'STARTING', 'RUNNING', 'STOPPING', 'ERROR'):
                state = 'UNKNOWN'
            with self._lock:
                self._state = state
                self._error = str(data.get('error', ''))
                self._unit = str(data.get('unit', ''))
                self._last_heartbeat = time.monotonic()
        except (TypeError, ValueError):
            self.get_logger().warning('Robot Manager status JSONを解釈できません')

    def _status_callback(self, message):
        self._decode_status(message.data)

    def _request_initial_status(self):
        if self._status_client.service_is_ready():
            future = self._status_client.call_async(Trigger.Request())
            future.add_done_callback(self._service_response)
            self._status_probe_timer.cancel()

    def _service_response(self, future):
        try:
            response = future.result()
            with self._lock:
                self._last_response = '' if response.message.startswith('{') else response.message
            if response.success and response.message.startswith('{'):
                self._decode_status(response.message)
        except Exception as exc:
            with self._lock:
                self._last_response = str(exc)

    def snapshot(self):
        # rosbagが異常終了した場合も画面の状態をRECORDINGのまま残さない。
        with self._lock:
            for item in self._recordings.values():
                if item['state'] == 'RECORDING' and item['process'].poll() is not None:
                    item['state'] = 'STOPPED'
                    item['error'] = 'rosbag processが終了しました (exit={})'.format(
                        item['process'].returncode)
        services = {name for name, _types in self.get_service_names_and_types()}
        service_seen = bool(services.intersection(self._service_names))
        service_ready = (self._start_client.service_is_ready()
                         and self._stop_client.service_is_ready()
                         and self._status_client.service_is_ready())
        with self._lock:
            last_heartbeat, state = self._last_heartbeat, self._state
            error, unit, response = self._error, self._unit, self._last_response
            telemetry = dict(self._telemetry)
            image, image_stamp = self._camera_image, self._camera_stamp
            image_received_stamp = self._camera_received_stamp
            camera_enabled, camera_error = self._camera_enabled, self._camera_error
            camera_subscription_active = self._camera_subscription is not None
            camera_subscription_started = self._camera_subscription_started
            camera_receive_count = self._camera_receive_count
            camera_converted_count = self._camera_converted_count
            camera_conversion_error_count = self._camera_conversion_error_count
            records = {name: dict(item) for name, item in self._recordings.items()}
        now = time.monotonic()
        age = None if last_heartbeat is None else now - last_heartbeat
        connection = ('CONNECTED' if service_ready and age is not None and age <= self.stale_timeout
                      else 'STALE' if service_seen else 'DISCONNECTED')
        data = {}
        for key, (value, stamp) in telemetry.items():
            data[key] = {'value': value, 'age': now - stamp,
                         'fresh': now - stamp <= self.topic_timeout}
        external = False
        external_name = self._recording_config.get('external_recorder_node_name', 'rosbag2_recorder')
        try:
            external = any(name == external_name for name in self.get_node_names())
        except Exception:
            pass
        disk_path = os.path.expanduser(self._recording_config.get(
            'output_directory', '/workspaces/patasmonkey_ws/bags/gui'))
        try:
            existing_path = disk_path
            while not os.path.exists(existing_path):
                parent = os.path.dirname(existing_path)
                if parent == existing_path:
                    existing_path = '.'
                    break
                existing_path = parent
            free_bytes = shutil.disk_usage(existing_path).free
        except OSError:
            free_bytes = None
        return {
            'connection': connection, 'core_state': state, 'error': error, 'unit': unit,
            'heartbeat_age_sec': age, 'last_response': response, 'telemetry': data,
            'camera': {
                'enabled': camera_enabled,
                'subscribed': camera_subscription_active,
                'image': image,
                'age': None if image_received_stamp is None else now-image_received_stamp,
                'subscription_age': (None if camera_subscription_started is None
                                     else now-camera_subscription_started),
                'converted_age': None if image_stamp is None else now-image_stamp,
                'receive_count': camera_receive_count,
                'converted_count': camera_converted_count,
                'conversion_error_count': camera_conversion_error_count,
                'error': camera_error,
                'topic': self._camera_config.get('topic', '/oak/color/image_raw'),
            },
            'recordings': records, 'external_recording': external,
            'free_bytes': free_bytes, 'recording_directory': disk_path,
        }

    def _mock_status_json(self):
        return json.dumps({'state': self._mock_state, 'error': self._error,
                           'unit': 'mock://robot-core', 'stamp_unix_sec': time.time()},
                          ensure_ascii=False)

    def _mock_publish(self):
        message = String()
        message.data = self._mock_status_json()
        self._decode_status(message.data)
        publisher = getattr(self, '_mock_publisher', None)
        if publisher is None:
            self._mock_publisher = self.create_publisher(String, self.status_topic, 10)
            publisher = self._mock_publisher
        publisher.publish(message)

    def _mock_tick(self):
        if self._mock_deadline is not None and time.monotonic() >= self._mock_deadline:
            self._mock_state, self._mock_deadline, self._mock_target = self._mock_target, None, None
        self._mock_publish()

    def _mock_start(self, _request, response):
        if self._mock_state in ('STARTING', 'STOPPING'):
            response.success, response.message = False, '遷移中です'
        elif self._mock_state == 'RUNNING':
            response.success, response.message = True, '既にRUNNINGです'
        else:
            self._mock_state, self._mock_target = 'STARTING', 'RUNNING'
            self._mock_deadline = time.monotonic() + 1.5
            response.success, response.message = True, 'mock START accepted'
        self._mock_publish()
        return response

    def _mock_stop(self, _request, response):
        if self._mock_state in ('STARTING', 'STOPPING'):
            response.success, response.message = False, '遷移中です'
        elif self._mock_state == 'STOPPED':
            response.success, response.message = True, '既にSTOPPEDです'
        else:
            self._mock_state, self._mock_target = 'STOPPING', 'STOPPED'
            self._mock_deadline = time.monotonic() + 1.5
            response.success, response.message = True, 'mock STOP accepted'
        self._mock_publish()
        return response

    def _mock_get_status(self, _request, response):
        response.success, response.message = True, self._mock_status_json()
        return response

    def request_mock_error(self):
        if self.backend_mode != 'mock' or not self._mock_error_client.service_is_ready():
            return False
        self._mock_error_client.call_async(Trigger.Request())
        return True

    def _mock_error_request(self, _request, response):
        self._mock_state, self._mock_target, self._mock_deadline = 'ERROR', None, None
        self._error = '開発用mockがERROR状態を表示しています'
        response.success, response.message = True, 'mock ERROR set'
        self._mock_publish()
        return response

    def stop_recordings_for_exit(self):
        """GUI終了時に所有bagへSIGINTを送り、metadataを閉じてから復帰する。"""
        with self._lock:
            active = [name for name, item in self._recordings.items()
                      if item['state'] in ('RECORDING', 'STOPPING')]
        for name in active:
            self.stop_recording(name)
        deadline = time.monotonic() + float(self._recording_config.get('stop_timeout_sec', 12.0)) + 4.0
        while time.monotonic() < deadline:
            with self._lock:
                if not any(item['state'] in ('RECORDING', 'STOPPING')
                           for item in self._recordings.values()):
                    break
            time.sleep(0.1)
