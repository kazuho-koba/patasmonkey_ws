"""ROS通信とテレメトリを集約し、Jetson上のRobot Managerへ操作を依頼する。"""

import json
import math
import threading
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, Joy, MagneticField, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .magnetic_heading import magnetic_bearing

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
        self._bag_service_names = {
            profile: {
                'start': manager.get(
                    profile+'_bag_start_service',
                    '/pm/robot_manager/{}_bag/start'.format(profile)),
                'stop': manager.get(
                    profile+'_bag_stop_service',
                    '/pm/robot_manager/{}_bag/stop'.format(profile)),
            }
            for profile in ('mission', 'debug')
        }
        self.stale_timeout = float(manager.get('stale_timeout_sec', 3.0))
        topics = config.get('topics', {})
        self.topic_timeout = float(topics.get('stale_timeout_sec', 2.0))
        self._lock = threading.RLock()
        self._state = 'UNKNOWN'
        self._error = ''
        self._unit = ''
        self._last_heartbeat = None
        self._last_response = ''
        self._service_names = {
            self.start_service_name, self.stop_service_name,
            self.get_status_service_name,
        }
        for services in self._bag_service_names.values():
            self._service_names.update(services.values())
        self._telemetry = {}
        self._compass_config = config.get('compass', {})
        self._wit_attitude = None
        map_config = config.get('map', {})
        path_limit = max(10, int(map_config.get('gnss_history_points', 2000)))
        self._gnss_path = deque(maxlen=path_limit)
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
        self._recordings = {
            profile: {
                'state': 'UNKNOWN', 'output': '', 'started_at_unix_sec': None,
                'elapsed_sec': 0.0, 'verified': None, 'error': '',
                'manager_error': '', 'free_bytes': None,
            }
            for profile in ('mission', 'debug')
        }
        self._status_subscription = self.create_subscription(
            String, self.status_topic, self._status_callback, 10)
        self._start_client = self.create_client(Trigger, self.start_service_name)
        self._stop_client = self.create_client(Trigger, self.stop_service_name)
        self._status_client = self.create_client(Trigger, self.get_status_service_name)
        self._bag_clients = {
            profile: {
                operation: self.create_client(Trigger, service_name)
                for operation, service_name in services.items()
            }
            for profile, services in self._bag_service_names.items()
        }
        self._telemetry_subscriptions = []
        # 絶対方位はodometryの初期yawではなく、Witの磁気ベクトルから計算する。
        self._add_subscription(Imu, topics.get('wit_imu', '/wit/imu'),
                               self._wit_imu_callback, qos=qos_profile_sensor_data)
        self._add_subscription(MagneticField, topics.get('wit_mag', '/wit/mag'),
                               self._wit_mag_callback, qos=qos_profile_sensor_data)
        self._add_subscription(Odometry, topics.get('odometry', '/odometry/global'),
                               self._odometry_callback)
        self._add_subscription(
            Odometry, topics.get('wheel_odometry', '/wheel/odometry'),
            self._wheel_odometry_callback, qos=qos_profile_sensor_data)
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
            self._mock_bag_deadlines = {profile: None for profile in ('mission', 'debug')}
            self._mock_bag_targets = {profile: None for profile in ('mission', 'debug')}
            self._mock_error_mode = False
            for item in self._recordings.values():
                item['state'] = 'STOPPED'
            self.create_service(Trigger, self.start_service_name, self._mock_start)
            self.create_service(Trigger, self.stop_service_name, self._mock_stop)
            self.create_service(Trigger, self.get_status_service_name, self._mock_get_status)
            for profile, services in self._bag_service_names.items():
                self.create_service(
                    Trigger, services['start'],
                    lambda request, response, bag=profile:
                        self._mock_start_bag(bag, request, response))
                self.create_service(
                    Trigger, services['stop'],
                    lambda request, response, bag=profile:
                        self._mock_stop_bag(bag, request, response))
            self.create_service(Trigger, '/pm/gui/mock_error', self._mock_error_request)
            self._mock_error_client = self.create_client(Trigger, '/pm/gui/mock_error')
            self.create_timer(0.25, self._mock_tick)
        else:
            self._status_probe_timer = self.create_timer(1.0, self._request_initial_status)

    def _add_subscription(self, msg_type, topic, callback, qos=10):
        if topic:
            self._telemetry_subscriptions.append(
                self.create_subscription(msg_type, topic, callback, qos))

    def _store(self, key, value):
        with self._lock:
            self._telemetry[key] = (value, time.monotonic())

    def _wheel_odometry_callback(self, message):
        """車体座標のvx [m/s]を保持する。指令値・融合odometryは使用しない。"""
        vx = float(message.twist.twist.linear.x)
        self._store('wheel_odometry', {'vx': vx if math.isfinite(vx) else None})

    def _wit_imu_callback(self, message):
        """磁気方位の傾斜補償用roll/pitchを保持。orientation yawは使用しない。"""
        q = message.orientation
        values = (q.x, q.y, q.z, q.w)
        norm = math.sqrt(sum(v*v for v in values))
        with self._lock:
            self._wit_attitude = None
            if (message.orientation_covariance[0] < 0 or not math.isfinite(norm)
                    or abs(norm-1.0) > 0.01):
                return
            roll, pitch, _yaw = _quaternion_rpy(q)
            self._wit_attitude = (roll, pitch, message.header.frame_id, time.monotonic())

    def _wit_mag_callback(self, message):
        """鮮度・frame・非ゼロ磁場を検査し、誤った0度表示を避ける。"""
        with self._lock:
            attitude = self._wit_attitude
        value = {'bearing': None, 'error': '', 'source': '/wit/mag',
                 'calibrated': bool(self._compass_config.get('calibrated', False))}
        try:
            if attitude is None or time.monotonic()-attitude[3] > self.topic_timeout:
                raise ValueError('Wit姿勢が未受信・無効・stale')
            if not message.header.frame_id or message.header.frame_id != attitude[2]:
                raise ValueError('Wit IMUと磁気のframeが一致しません')
            field = message.magnetic_field
            cfg = self._compass_config
            value['bearing'] = magnetic_bearing(
                (field.x,field.y,field.z), attitude[0], attitude[1],
                cfg.get('bias', (0,0,0)), cfg.get('scale', (1,1,1)),
                float(cfg.get('offset_deg', 0)), float(cfg.get('declination_deg', 0)))
        except (ValueError, TypeError) as exc:
            value['error'] = str(exc)
        self._store('magnetic_heading', value)

    def _odometry_callback(self, message):
        pose = message.pose.pose
        roll, pitch, yaw = _quaternion_rpy(pose.orientation)
        self._store('odometry', {
            'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'frame_id': message.header.frame_id,
            'child_frame_id': message.child_frame_id,
        })

    def _joy_callback(self, message):
        self._store('joy', {'buttons': list(message.buttons), 'axes': list(message.axes)})

    def _fix_callback(self, message):
        latitude, longitude = float(message.latitude), float(message.longitude)
        self._store('fix', {
            'status': int(message.status.status), 'latitude': latitude,
            'longitude': longitude, 'altitude': float(message.altitude),
        })
        # Web地図の軌跡にはstatusが有効で範囲内のWGS84 fixだけを蓄積する。
        if (int(message.status.status) < 0 or not math.isfinite(latitude)
                or not math.isfinite(longitude) or abs(latitude) > 90.0
                or abs(longitude) > 180.0):
            return
        with self._lock:
            if self._gnss_path:
                previous_lat, previous_lon = self._gnss_path[-1]
                mean_lat = math.radians((previous_lat + latitude) * 0.5)
                east_m = math.radians(longitude - previous_lon) * math.cos(mean_lat) * 6371000.0
                north_m = math.radians(latitude - previous_lat) * 6371000.0
                # GNSSの小さな揺れで同じ場所の点を増やさず、0.5 m以上の移動だけ記録する。
                if math.hypot(east_m, north_m) < 0.5:
                    return
            self._gnss_path.append((latitude, longitude))

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
        if profile not in self._bag_clients:
            return False, '未対応のbag profileです: {}'.format(profile)
        ok = self._request(self._bag_clients[profile]['start'])
        return ok, 'Jetson上の{} bag開始を要求しました'.format(profile)

    def stop_recording(self, profile):
        if profile not in self._bag_clients:
            return False
        return self._request(self._bag_clients[profile]['stop'])

    def request_start(self):
        return self._request(self._start_client)

    def request_stop(self):
        """Jetson managerへCore停止を依頼し、bagの保存確認はmanagerに任せる。"""
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
                remote_recordings = data.get('recordings', {})
                if isinstance(remote_recordings, dict):
                    for profile in ('mission', 'debug'):
                        item = remote_recordings.get(profile)
                        if isinstance(item, dict):
                            self._recordings[profile] = dict(item)
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
        services = {name for name, _types in self.get_service_names_and_types()}
        service_seen = bool(services.intersection(self._service_names))
        bag_services_ready = all(
            client.service_is_ready()
            for clients in self._bag_clients.values()
            for client in clients.values())
        service_ready = (self._start_client.service_is_ready()
                         and self._stop_client.service_is_ready()
                         and self._status_client.service_is_ready()
                         and bag_services_ready)
        with self._lock:
            last_heartbeat, state = self._last_heartbeat, self._state
            error, unit, response = self._error, self._unit, self._last_response
            telemetry = dict(self._telemetry)
            gnss_path = list(self._gnss_path)
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
        active_records = [item for item in records.values()
                          if item.get('state') in ('RUNNING', 'STARTING', 'STOPPING')]
        external = bool(active_records)
        outputs = [str(item.get('output')) for item in records.values()
                   if item.get('output')]
        disk_path = ', '.join(outputs) if outputs else 'Jetson bag output: 未取得'
        free_values = [item.get('free_bytes') for item in records.values()
                       if item.get('free_bytes') is not None]
        free_bytes = min(free_values) if free_values else None
        return {
            'connection': connection, 'core_state': state, 'error': error, 'unit': unit,
            'heartbeat_age_sec': age, 'last_response': response, 'telemetry': data,
            'gnss_path': gnss_path,
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
        now = time.time()
        recordings = {}
        for profile, item in self._recordings.items():
            record = dict(item)
            started = record.get('started_at_unix_sec')
            record.update({
                'state': record.get('state', 'STOPPED'),
                'unit': 'mock://{}-bag'.format(profile),
                'elapsed_sec': max(
                    0.0, (now if record.get('state') == 'RUNNING'
                          else float(record.get('stopped_at_unix_sec') or now))-started)
                if started else 0.0,
                'free_bytes': 20 * 1024 ** 3,
            })
            recordings[profile] = record
        return json.dumps({
            'state': self._mock_state, 'error': self._error,
            'unit': 'mock://robot-core', 'recordings': recordings,
            'stamp_unix_sec': now,
        }, ensure_ascii=False)

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
        now = time.monotonic()
        for profile, deadline in self._mock_bag_deadlines.items():
            if deadline is not None and now >= deadline:
                target = self._mock_bag_targets[profile]
                item = self._recordings[profile]
                item['state'] = target
                item['verified'] = target == 'STOPPED'
                item['error'] = ''
                if target == 'STOPPED':
                    item['stopped_at_unix_sec'] = time.time()
                self._mock_bag_deadlines[profile] = None
                self._mock_bag_targets[profile] = None
        self._mock_publish()

    def _mock_start(self, _request, response):
        if self._mock_state in ('STARTING', 'STOPPING'):
            response.success, response.message = False, '遷移中です'
        elif self._mock_state == 'RUNNING':
            response.success, response.message = True, '既にRUNNINGです'
        else:
            self._mock_state, self._mock_target = 'STARTING', 'RUNNING'
            self._mock_deadline = time.monotonic() + 1.5
            # GUIからのCore起動ではbagを変更せず、独立したSTART操作を待つ。
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
            for profile, item in self._recordings.items():
                if item.get('state') in ('RUNNING', 'STARTING', 'STOPPING'):
                    item['state'] = 'STOPPING'
                    self._mock_bag_targets[profile] = 'STOPPED'
                    self._mock_bag_deadlines[profile] = time.monotonic() + 1.0
            response.success, response.message = True, 'mock STOP accepted'
        self._mock_publish()
        return response

    def _mock_start_bag(self, profile, _request, response):
        item = self._recordings[profile]
        if self._mock_state != 'RUNNING':
            response.success, response.message = False, 'Robot CoreがSTOPPEDです'
        elif item.get('state') == 'RUNNING':
            response.success, response.message = True, '既にRECORDINGです'
        elif item.get('state') in ('STARTING', 'STOPPING'):
            response.success, response.message = False, 'bag遷移中です'
        else:
            item.update({
                'state': 'STARTING',
                'output': 'mock://jetson/bags/{}/{}_mock'.format(profile, profile),
                'started_at_unix_sec': time.time(),
                'stopped_at_unix_sec': None,
                'verified': None,
                'error': '',
            })
            self._mock_bag_targets[profile] = 'RUNNING'
            self._mock_bag_deadlines[profile] = time.monotonic() + 1.0
            response.success, response.message = True, 'mock bag START accepted'
        self._mock_publish()
        return response

    def _mock_stop_bag(self, profile, _request, response):
        item = self._recordings[profile]
        if item.get('state') == 'STOPPED':
            response.success, response.message = True, 'bagは既にSTOPPEDです'
        elif item.get('state') in ('STARTING', 'STOPPING'):
            response.success, response.message = False, 'bag遷移中です'
        else:
            item['state'] = 'STOPPING'
            self._mock_bag_targets[profile] = 'STOPPED'
            self._mock_bag_deadlines[profile] = time.monotonic() + 1.0
            response.success, response.message = True, 'mock bag STOP accepted'
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
        """GUI終了時もJetson上のCoreとbagは継続させる。"""
        return
