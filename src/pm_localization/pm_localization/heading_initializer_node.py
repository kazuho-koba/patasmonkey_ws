#!/usr/bin/env python3
"""静止時の補正磁場から一度だけ両EKFを初期化し、GNSS変換を検証する。"""
import copy
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from robot_localization.srv import SetDatum, SetPose, FromLL
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def wrap_pi(x):
    return math.atan2(math.sin(x), math.cos(x))


def rpy_from_quaternion(q):
    roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
    pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return roll, pitch, yaw


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def magnetic_yaw(field, roll, pitch, offset_deg, declination_deg, min_horizontal=1.0):
    """車体x前/y左/z上の磁場を水平化し、北0時計回りからENU yawへ変換。

    入力fieldはbias/scale補正済みのraw register単位。offsetは真北基準を含む。
    """
    x, y, z = field
    if not all(math.isfinite(v) for v in (x, y, z, roll, pitch, offset_deg, declination_deg)):
        raise ValueError('磁気または姿勢が非有限')
    hx = math.cos(pitch)*x + math.sin(pitch)*(math.sin(roll)*y + math.cos(roll)*z)
    hy = math.cos(roll)*y - math.sin(roll)*z
    if math.hypot(hx, hy) < min_horizontal:
        raise ValueError('水平磁場が小さすぎる')
    bearing = math.atan2(hy, hx) + math.radians(offset_deg + declination_deg)
    return wrap_pi(math.pi/2 - bearing)


def circular_mean(values):
    """円平均と円標準偏差。±π境界の方向を正しく平均する。"""
    s = sum(math.sin(v) for v in values) / len(values)
    c = sum(math.cos(v) for v in values) / len(values)
    return math.atan2(s, c), math.sqrt(max(0., -2*math.log(max(math.hypot(s, c), 1e-12))))


def stamp_key(message):
    return (message.header.stamp.sec, message.header.stamp.nanosec)


def fresh_stamp(message, now, max_age):
    """ROS clockとheader時刻の差を使い、古い値の繰返しや未来値を拒否する。"""
    stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
    return stamp > 0 and -.2 <= now-stamp <= max_age


class HeadingInitializerNode(Node):
    """停止中に方位を取得し、応答と出力を確認して段階的に完了する。"""
    def __init__(self):
        super().__init__('heading_initializer_node')
        defaults = dict(startup_settle_sec=1., average_duration_sec=1., average_max_sec=3.,
                        input_timeout_sec=.5, wheel_timeout_sec=.5,
                        stationary_speed_threshold_mps=.03,
                        stationary_yaw_rate_threshold_rad_s=.05,
                        max_initial_yaw_stddev_radians=.10, min_horizontal_field=1.,
                        max_pitch_rad=1., initial_yaw_variance=.122,
                        service_timeout_sec=5., verify_yaw_tolerance_rad=.15,
                        odom_timeout_sec=1., offset_deg=0., declination_deg=0.,
                        require_gnss=True, yaw_correction_radians=0.0,
                        input_imu_topic='/wit/imu',
                        output_imu_topic='/wit/imu/heading_calibrated',
                        wheel_odom_topic='/wheel/odometry', gnss_fix_topic='/fix/gated',
                        local_set_pose_service='/localization/local/set_pose',
                        navsat_set_datum_service='/datum', initial_sample_count=100,
                        initialization_timeout_sec=0.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if not 0 < self.p['average_duration_sec'] <= self.p['average_max_sec'] <= 3.0:
            raise ValueError('平均時間は0秒より長く、最大3秒以内に設定してください')
        if self.p['initial_sample_count'] < 1:
            raise ValueError('initial_sample_countは1以上に設定してください')
        self.started_at = self.now()
        self.state = 'WAITING'
        self.reason = '磁気・IMU・車輪の停止情報を待機'
        self.error = ''
        self.seed_yaw = None
        self.sample_std = None
        self.stationary_since = None
        self.samples = deque()
        self.imu_cache = {}
        self.mag_cache = {}
        self.wheel = None
        self.last_imu = None
        self.last_mag = None
        self.local = None
        self.global_odom = None
        self.fix = None
        self.gps = None
        self.pending = None
        self.deadline = 0.
        self.expected = None
        self.verify_after = 0.
        self.datum = None
        self.ll_futures = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.frame_checked_at = 0.
        self.checked_frame = ''
        self.status_pub = self.create_publisher(String, '/localization/initialization/status', 10)
        self.gps_pub = self.create_publisher(Odometry, '/odometry/gps', 10)
        # 旧観測topicは互換性のため保持する。新しい初期化・navsatには使用しない。
        self.legacy_imu_pub = self.create_publisher(Imu, self.p['output_imu_topic'], 50)
        self.create_subscription(Imu, self.p['input_imu_topic'], self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(MagneticField, '/wit/mag_corrected', self.mag_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['wheel_odom_topic'], self.wheel_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/local_horizontal', self.local_callback, 20)
        self.create_subscription(Odometry, '/odometry/global', self.global_callback, 20)
        self.create_subscription(Odometry, '/odometry/gps/unverified', self.gps_callback, 10)
        self.create_subscription(NavSatFix, self.p['gnss_fix_topic'], self.fix_callback, qos_profile_sensor_data)
        self.local_service = self.create_client(SetPose, self.p['local_set_pose_service'])
        self.global_service = self.create_client(SetPose, '/localization/global/set_pose')
        self.datum_service = self.create_client(SetDatum, self.p['navsat_set_datum_service'])
        self.fromll_service = self.create_client(FromLL, '/fromLL')
        self.create_timer(.1, self.tick)
        self.create_timer(1., self.publish_status)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, state, reason):
        changed = (state, reason) != (self.state, self.reason)
        if state != self.state and state in ('LOCAL_SERVICE', 'GLOBAL_SERVICE', 'DATUM_SERVICE'):
            self.deadline = self.now() + self.p['service_timeout_sec']
        if changed:
            self.get_logger().info('%s: %s' % (state, reason))
        self.state, self.reason = state, reason
        # 同じ状態を毎IMU sampleでpublishするとbagとDDSの負荷が増える。
        # 状態変化時と1 Hz timerの両方で表示の鮮度を維持する。
        if changed:
            self.publish_status()

    def fail(self, reason):
        self.error = reason
        self.set_state('ERROR', reason + '。localizationを再起動してください')

    def publish_status(self):
        message = String()
        message.data = json.dumps(dict(state=self.state, reason=self.reason, error=self.error,
                                       seed_yaw_rad=self.seed_yaw, yaw_std_rad=self.sample_std,
                                       require_gnss=bool(self.p['require_gnss'])),
                                  ensure_ascii=False)
        self.status_pub.publish(message)

    def wheel_callback(self, msg):
        self.wheel = (msg, self.now())

    def local_callback(self, msg):
        self.local = (msg, self.now())

    def global_callback(self, msg):
        self.global_odom = (msg, self.now())

    def fix_callback(self, msg):
        if msg.status.status >= 0 and all(math.isfinite(x) for x in (
                msg.latitude, msg.longitude, msg.altitude)) and (
                -90. <= msg.latitude <= 90. and -180. <= msg.longitude <= 180.):
            self.fix = (msg, self.now())

    def gps_callback(self, msg):
        self.gps = (msg, self.now())
        if self.state == 'READY':
            self.gps_pub.publish(msg)

    def imu_callback(self, msg):
        now = self.now()
        output = copy.deepcopy(msg)
        roll, pitch, yaw = rpy_from_quaternion(msg.orientation)
        q = quaternion_from_rpy(roll, pitch, wrap_pi(yaw+self.p['yaw_correction_radians']))
        output.orientation.x, output.orientation.y, output.orientation.z, output.orientation.w = q
        self.legacy_imu_pub.publish(output)
        self.last_imu = (msg, now)
        self.imu_cache[stamp_key(msg)] = msg
        self.pair(stamp_key(msg))
        if len(self.imu_cache) > 50:
            self.imu_cache.pop(next(iter(self.imu_cache)))

    def mag_callback(self, msg):
        self.last_mag = (msg, self.now())
        self.mag_cache[stamp_key(msg)] = msg
        self.pair(stamp_key(msg))
        if len(self.mag_cache) > 50:
            self.mag_cache.pop(next(iter(self.mag_cache)))

    def stationary(self):
        """車輪の前後速度・旋回速度とIMU全角速度を鮮度付きで確認する。"""
        now = self.now()
        if self.wheel is None or now-self.wheel[1] > self.p['wheel_timeout_sec']:
            return False, '新鮮な車輪odometry待ち（モータドライバ状態を確認）'
        if self.last_imu is None or now-self.last_imu[1] > self.p['input_timeout_sec']:
            return False, '新鮮なIMU待ち'
        msg = self.wheel[0]
        imu = self.last_imu[0]
        if not fresh_stamp(msg, now, self.p['wheel_timeout_sec']):
            return False, '車輪odometryのheader時刻が古いか未来'
        if not fresh_stamp(imu, now, self.p['input_timeout_sec']):
            return False, 'IMUのheader時刻が古いか未来'
        speed = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        gyro = imu.angular_velocity
        vals = (speed, wz, gyro.x, gyro.y, gyro.z)
        if not all(math.isfinite(v) for v in vals):
            return False, '車輪速度または角速度が無効'
        if (abs(speed) > self.p['stationary_speed_threshold_mps'] or
                abs(wz) > self.p['stationary_yaw_rate_threshold_rad_s'] or
                max(abs(gyro.x), abs(gyro.y), abs(gyro.z)) >
                self.p['stationary_yaw_rate_threshold_rad_s']):
            return False, '車体の移動・回転を検出'
        return True, ''

    def pair(self, key):
        if key not in self.imu_cache or key not in self.mag_cache:
            return
        imu, mag = self.imu_cache.pop(key), self.mag_cache.pop(key)
        if self.state not in ('WAITING', 'COLLECTING'):
            return
        stationary, reason = self.stationary()
        if not stationary:
            self.stationary_since = None
            self.samples.clear()
            self.set_state('WAITING', reason)
            return
        if not fresh_stamp(mag, self.now(), self.p['input_timeout_sec']):
            self.stationary_since = None
            self.samples.clear()
            self.set_state('WAITING', '磁気header時刻が古いか未来')
            return
        if not imu.header.frame_id or imu.header.frame_id != mag.header.frame_id:
            self.stationary_since = None
            self.samples.clear()
            self.set_state('WAITING', '磁気とIMUのframeが不一致')
            return
        if imu.header.frame_id != self.checked_frame or self.now()-self.frame_checked_at > .5:
            try:
                transform = self.tf_buffer.lookup_transform('base_link', imu.header.frame_id, Time())
                rotation = transform.transform.rotation
                # 現行のGUI式はIMU軸とbase軸の一致が前提。異なる取り付けの
                # 場合は未検証の軸変換を暗黙に適用せず初期化を保留する。
                if math.sqrt(rotation.x**2+rotation.y**2+rotation.z**2) > .005:
                    raise ValueError('IMU取付回転は未対応')
                self.checked_frame = imu.header.frame_id
                self.frame_checked_at = self.now()
            except Exception as exc:
                self.stationary_since = None
                self.samples.clear()
                self.set_state('WAITING', 'IMU取付TFを確認できません: {}'.format(exc))
                return
        q = imu.orientation
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if imu.orientation_covariance[0] < 0 or not math.isfinite(norm) or abs(norm-1) > .01:
            self.stationary_since = None
            self.samples.clear()
            self.set_state('WAITING', 'IMU姿勢が無効')
            return
        roll, pitch, _ = rpy_from_quaternion(q)
        try:
            if abs(pitch) > self.p['max_pitch_rad']:
                raise ValueError('pitchが許容範囲外')
            field = mag.magnetic_field
            yaw = magnetic_yaw((field.x, field.y, field.z), roll, pitch,
                               self.p['offset_deg'], self.p['declination_deg'],
                               self.p['min_horizontal_field'])
        except ValueError as exc:
            self.stationary_since = None
            self.samples.clear()
            self.set_state('WAITING', str(exc))
            return
        now = self.now()
        if self.stationary_since is None:
            self.stationary_since = now
        if now-self.stationary_since < self.p['startup_settle_sec']:
            self.set_state('WAITING', '停止・センサの安定化待ち')
            return
        self.samples.append((now, yaw))
        while self.samples and now-self.samples[0][0] > self.p['average_max_sec']:
            self.samples.popleft()
        self.set_state('COLLECTING', '停止中の磁気方位を平均中')
        if (now-self.samples[0][0] < self.p['average_duration_sec'] or
                len(self.samples) < self.p['initial_sample_count']):
            return
        mean, std = circular_mean([v for _, v in self.samples])
        if std > self.p['max_initial_yaw_stddev_radians']:
            self.set_state('COLLECTING', '磁気方位のばらつきが大きいため待機')
            return
        self.seed_yaw, self.sample_std = mean, std
        self.set_state('LOCAL_SERVICE', 'local EKFの初期化を待機')

    def pose_request(self, item, frame):
        """最新poseの位置・tiltを保持してyawだけを置換する。速度はEKF側でリセット。"""
        q = item.pose.pose.orientation
        pos = item.pose.pose.position
        values = (q.x, q.y, q.z, q.w, pos.x, pos.y, pos.z)
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if (item.header.frame_id != frame or not all(math.isfinite(x) for x in values)
                or abs(norm-1.0) > .01):
            raise ValueError('EKFの姿勢・位置またはframeが不正')
        pose = copy.deepcopy(item.pose)
        roll, pitch, _ = rpy_from_quaternion(pose.pose.orientation)
        q = quaternion_from_rpy(roll, pitch, self.seed_yaw)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        pose.covariance[35] = float(self.p['initial_yaw_variance'])
        request = SetPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = frame
        request.pose.pose = pose
        return request

    def begin_call(self, client, request, state, reason):
        self.pending = client.call_async(request)
        self.deadline = self.now()+self.p['service_timeout_sec']
        self.set_state(state, reason)

    def poll_call(self, next_state, next_reason):
        if not self.pending.done():
            if self.now() > self.deadline:
                self.fail('サービス応答timeout')
            return
        try:
            self.pending.result()
        except Exception as exc:
            self.fail('サービス失敗: {}'.format(exc))
            return
        self.pending = None
        self.verify_after = self.now()
        self.deadline = self.now()+self.p['service_timeout_sec']
        self.set_state(next_state, next_reason)

    def verify_pose(self, item, next_state, reason):
        if (item is not None and item[1] > self.verify_after and
                fresh_stamp(item[0], self.now(), self.p['odom_timeout_sec'])):
            _, _, yaw = rpy_from_quaternion(item[0].pose.pose.orientation)
            if abs(wrap_pi(yaw-self.seed_yaw)) <= self.p['verify_yaw_tolerance_rad']:
                self.set_state(next_state, reason)
                return
        if self.now() > self.deadline:
            self.fail('EKF出力yawの反映を確認できません')

    def tick(self):
        now = self.now()
        if self.state in ('WAITING', 'COLLECTING'):
            timeout = self.p['initialization_timeout_sec']
            if timeout > 0 and now-self.started_at > timeout:
                self.fail('停止中の磁気方位を制限時間内に取得できません')
                return
            stationary, reason = self.stationary()
            if not stationary or self.last_mag is None or now-self.last_mag[1] > self.p['input_timeout_sec']:
                self.samples.clear()
                self.stationary_since = None
                self.set_state('WAITING', reason or '新鮮な補正磁気データ待ち')
            return
        if self.state in ('ERROR', 'READY'):
            return
        stationary, reason = self.stationary()
        if not stationary:
            self.fail('初期化途中に停止を確認できません: ' + reason)
            return
        if self.state == 'LOCAL_SERVICE':
            if (self.local and now-self.local[1] < self.p['odom_timeout_sec'] and
                    fresh_stamp(self.local[0], now, self.p['odom_timeout_sec']) and
                    self.local_service.service_is_ready()):
                try:
                    request = self.pose_request(self.local[0], 'odom')
                except ValueError as exc:
                    self.fail(str(exc))
                    return
                self.begin_call(self.local_service, request, 'LOCAL_PENDING', 'local EKFの応答待ち')
            elif now > self.deadline:
                self.fail('local EKFのserviceまたは新鮮な出力がありません')
        elif self.state == 'LOCAL_PENDING':
            self.poll_call('LOCAL_VERIFY', 'local EKF出力を確認')
        elif self.state == 'LOCAL_VERIFY':
            self.verify_pose(self.local, 'GLOBAL_SERVICE' if self.p['require_gnss'] else 'READY',
                             'global EKFの初期化待ち' if self.p['require_gnss'] else 'local初期化完了（GNSS無効）')
        elif self.state == 'GLOBAL_SERVICE':
            if (self.global_odom and now-self.global_odom[1] < self.p['odom_timeout_sec'] and
                    fresh_stamp(self.global_odom[0], now, self.p['odom_timeout_sec']) and
                    self.global_service.service_is_ready()):
                try:
                    request = self.pose_request(self.global_odom[0], 'map')
                except ValueError as exc:
                    self.fail(str(exc))
                    return
                self.begin_call(self.global_service, request, 'GLOBAL_PENDING', 'global EKFの応答待ち')
            elif now > self.deadline:
                self.fail('global EKFのserviceまたは新鮮な出力がありません')
        elif self.state == 'GLOBAL_PENDING':
            self.poll_call('GLOBAL_VERIFY', 'global EKF出力を確認')
        elif self.state == 'GLOBAL_VERIFY':
            self.verify_pose(self.global_odom, 'DATUM_SERVICE', '有効GNSSとdatum serviceを待機')
        elif self.state == 'DATUM_SERVICE':
            # 地図座標は東x/北y。datum yawに車体yawを設定すると地図自体が回転する。
            if self.fix and now-self.fix[1] < 2.5 and self.datum_service.service_is_ready():
                self.datum = self.fix[0]
                req = SetDatum.Request()
                req.geo_pose.position.latitude = self.datum.latitude
                req.geo_pose.position.longitude = self.datum.longitude
                req.geo_pose.position.altitude = self.datum.altitude
                req.geo_pose.orientation.w = 1.0
                self.begin_call(self.datum_service, req, 'DATUM_PENDING', 'GNSS datumの応答待ち')
            elif self.fix and now-self.fix[1] < 2.5 and now > self.deadline:
                self.fail('datum serviceが利用できません')
        elif self.state == 'DATUM_PENDING':
            self.poll_call('GPS_VERIFY', 'GNSS座標変換の出力を確認')
        elif self.state == 'GPS_VERIFY':
            if self.gps and self.gps[1] > self.verify_after and self.fromll_service.service_is_ready():
                gps_position = self.gps[0].pose.pose.position
                if (not all(math.isfinite(v) for v in (gps_position.x, gps_position.y)) or
                        math.hypot(gps_position.x, gps_position.y) > 15.0):
                    if now > self.deadline:
                        self.fail('datum後のGNSS位置が原点付近ではありません')
                    return
                # 緯度経度を東・北へそれぞれ約10 m移して地図軸を確かめる。
                lat, lon = self.datum.latitude, self.datum.longitude
                if abs(math.cos(math.radians(lat))) < .01:
                    self.fail('極域ではGNSS座標変換を検証できません')
                    return
                points = [(lat, lon), (lat, lon+10/(111320*math.cos(math.radians(lat)))),
                          (lat+10/111320, lon)]
                self.ll_futures = []
                for la, lo in points:
                    req = FromLL.Request()
                    req.ll_point.latitude, req.ll_point.longitude = la, lo
                    req.ll_point.altitude = self.datum.altitude
                    self.ll_futures.append(self.fromll_service.call_async(req))
                self.deadline = now+self.p['service_timeout_sec']
                self.set_state('AXES_VERIFY', 'GNSSの東北座標軸を確認')
            elif now > self.deadline:
                self.fail('datum後のGNSS変換出力を確認できません')
        elif self.state == 'AXES_VERIFY':
            if all(f.done() for f in self.ll_futures):
                try:
                    points = [f.result().map_point for f in self.ll_futures]
                    ex, ey = points[1].x-points[0].x, points[1].y-points[0].y
                    nx, ny = points[2].x-points[0].x, points[2].y-points[0].y
                    if math.hypot(points[0].x, points[0].y) > 1.0:
                        raise ValueError('datumの地図原点が一致しません')
                    if not (8 < ex < 12 and abs(ey) < 2 and 8 < ny < 12 and abs(nx) < 2):
                        raise ValueError('東北方向が期待する地図軸と一致しません')
                except Exception as exc:
                    self.fail('GNSS座標軸検証失敗: {}'.format(exc))
                    return
                self.set_state('READY', 'local・global・GNSSの初期化と座標軸を確認')
            elif now > self.deadline:
                self.fail('GNSS座標軸検証の応答timeout')


def main(args=None):
    rclpy.init(args=args)
    node = HeadingInitializerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
