#!/usr/bin/env python3
"""OpenVINSのpose・twist・共分散を同一の取付変換でbase_linkへ変換する。"""
from collections import deque
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from pm_localization.vio_rigid_transform import transform_pose, transform_twist, yaw_alignment
# 過去bagの監査と既存の評価CLIのimport先を維持する。実行時の変換には
# 以下の旧式のpose/twist補正を呼ばない。旧nodeの起動も別entry pointに限定する。
from pm_localization.vio_legacy_transform import (  # noqa: F401
    quat_to_rot, rot_to_quat, rpy_from_rot, yaw_to_rot, rad2deg, stamp_to_sec,
    wrap_to_pi, transform_to_matrix, odom_pose_to_matrix, openvins_odom_pose_to_matrix,
    skew, so3_exp, so3_log, cov_list_to_mat6, cov_mat6_to_list,
    transform_pose_covariance_ros_approx, calculate_corrected_output_rotation,
    correct_pose_covariance_for_output_rotation, transform_twist_and_covariance_ros_approx,
    twist_axis_fix_matrix3, twist_axis_fix_matrix6, correct_twist_velocity,
    correct_twist_covariance,
)


class VioOdomAdapterNode(Node):
    """固定の世界座標整列と固定の取付変換を使用するVIO adapter。

    poseはheader.frame_id、twistはchild_frame_idの成分でpublishする。
    共分散は取付・初期配置を条件とした値であり、それらの校正誤差や
    推定間の未公開相関を含む絶対精度の保証ではない。
    """
    def __init__(self, **kwargs):
        super().__init__('vio_odom_adapter_node', **kwargs)
        defaults = {
            'input_topic': '/ov_msckf/odomimu', 'output_topic': '/vio/odometry',
            'input_frame_id': 'global', 'input_child_frame_id': 'imu',
            'output_frame_id': 'odom', 'output_child_frame_id': 'base_link',
            'base_frame_id': 'base_link', 'oak_imu_frame_id': 'openvins_imu_link',
            'publish_tf': False, 'align_initial_to_tf': True, 'zero_initial_pose': True,
            'initial_reference_topic': '/odometry/local', 'reference_time_tolerance_sec': .1,
            'require_heading_ready': False,
            'initialization_status_topic': '/localization/initialization/status',
            'mount_rotation_source': 'parameter',
            'mount_rotation': [0., 0., -1., 0., -1., 0., -1., 0., 0.],
            'pose_covariance_orientation_frame': 'imu',
            'input_timeout_sec': 1., 'clock_reset_threshold_sec': 1.,
            'enable_diagnostics': False, 'diagnostics_interval_sec': 1.,
            'diagnostics_topic': '/vio/adapter/diagnostics',
            # 旧launchのparameter受理を維持するが、rigid変換では使わない。
            'invert_openvins_orientation': False,
            'align_output_orientation_to_initial_tf': False,
            'invert_relative_rotation': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if self.p['publish_tf']:
            raise ValueError('odom→base TFはEKFの担当。publish_tf=falseが必要')
        if self.p['output_child_frame_id'] != self.p['base_frame_id']:
            raise ValueError('output_child_frame_idとbase_frame_idを一致させてください')
        if self.p['mount_rotation_source'] not in ('tf', 'parameter'):
            raise ValueError('mount_rotation_sourceはtfまたはparameter')
        if self.p['pose_covariance_orientation_frame'] not in ('imu', 'global'):
            raise ValueError('pose_covariance_orientation_frameはimuまたはglobal')
        for name in ('reference_time_tolerance_sec', 'input_timeout_sec',
                     'clock_reset_threshold_sec', 'diagnostics_interval_sec'):
            if not math.isfinite(self.p[name]) or self.p[name] <= 0:
                raise ValueError(name + 'は有限正数で指定してください')
        if any(self.p[n] for n in ('invert_openvins_orientation',
               'align_output_orientation_to_initial_tf', 'invert_relative_rotation')):
            self.get_logger().warn('旧姿勢反転parameterは無視します。ROS quaternionと単一の剛体変換を使用します')
        self.mount = np.asarray(self.p['mount_rotation'], dtype=float).reshape(3, 3)
        # 回転行列・パラメータの誤りは、最初のdataを待たず起動時に検出する。
        transform_twist(np.zeros(6), np.eye(6), self.mount, np.zeros(3))
        self.T_base_imu = None
        self.A = None
        self.b = None
        self.references = deque(maxlen=300)  # 50Hzで約6秒、画像は保持しない。
        self.heading_ready = not self.p['require_heading_ready']
        self.reset_required = False
        self.last_stamp = None
        self.last_received = None
        self.last_warning = -math.inf
        self.received = self.forwarded = self.rejected = 0
        self.reason = '取付TFと初回入力待ち'
        self.state = 'waiting'
        self.alignment_stamp = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Odometry, self.p['output_topic'], 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, self.p['diagnostics_topic'], 10)
        self.create_subscription(Odometry, self.p['input_topic'], self.odom_callback, 50)
        self.create_subscription(Odometry, self.p['initial_reference_topic'], self.reference_callback, 50)
        self.create_subscription(String, self.p['initialization_status_topic'], self.heading_callback, 10)
        self.create_timer(self.p['diagnostics_interval_sec'], self.publish_diagnostics)
        self.get_logger().info('rigid VIO adapter起動。取付回転のsource=' + self.p['mount_rotation_source'])

    def reference_callback(self, msg):
        """初回整列用のlocal poseを有界履歴に保持。以後の追従には使わない。"""
        self.references.append(msg)

    def heading_callback(self, msg):
        """地球基準yawのset_poseより前にVIO世界座標を固定しない。"""
        if not self.p['require_heading_ready']:
            return
        try:
            state = json.loads(msg.data)['state']
        except (ValueError, KeyError, TypeError):
            return
        if self.A is not None and self.heading_ready and state != 'READY':
            # 動作中の再初期化でlocalの世界座標が変わる場合、黙ってVIOを
            # 再配置するとposeが飛ぶため明示的に保護状態へ入る。
            self.reset_required = True
            self.reason = 'heading初期化状態がREADYから変化。adapter再起動が必要'
        self.heading_ready = state == 'READY'

    def _mount(self):
        if self.T_base_imu is not None:
            return self.T_base_imu
        try:
            msg = self.tf_buffer.lookup_transform(self.p['base_frame_id'],
                    self.p['oak_imu_frame_id'], rclpy.time.Time())
            self._check_quaternion(msg.transform.rotation)
            matrix = transform_to_matrix(msg)
        except TransformException:
            self.reason = 'base→IMUの固定TF待ち'
            return None
        if self.p['mount_rotation_source'] == 'parameter':
            # parameterは実効R_base_imuそのもの。TF回転へ追加乗算しないため、
            # 後日TFを校正しても、同じ回転を二度適用しない。
            matrix[:3, :3] = self.mount
        transform_twist(np.zeros(6), np.eye(6), matrix[:3, :3], matrix[:3, 3])
        self.T_base_imu = matrix
        return matrix

    def _initialize(self, position, rotation, mount, stamp):
        if self.A is not None:
            return True
        if not self.heading_ready:
            self.reason = 'heading初期化のREADY待ち'
            return False
        if self.p['align_initial_to_tf']:
            # 最新TFの無時刻lookupではなく、raw headerと近いlocal poseを使う。
            # 同じフレームのlocal出力はodom→base TFと同じposeである。
            references = [m for m in self.references
                if m.header.frame_id == self.p['output_frame_id']
                and m.child_frame_id == self.p['output_child_frame_id']]
            if not references:
                self.reason = '初回整列用local odometry待ち'
                return False
            reference = min(references, key=lambda m: abs(stamp_to_sec(m.header.stamp)-stamp))
            if abs(stamp_to_sec(reference.header.stamp)-stamp) > self.p['reference_time_tolerance_sec']:
                self.reason = '初回local参照のheader時刻差が許容範囲外'
                return False
            q = reference.pose.pose.orientation
            self._check_quaternion(q)
            reference_position = reference.pose.pose.position
            reference_position = np.array([reference_position.x, reference_position.y, reference_position.z])
            reference_rotation = quat_to_rot(q)
        else:
            # 独立VIOの原点はodomとは異なる。独立frameを明示させる。
            if self.p['output_frame_id'] == 'odom':
                raise ValueError('align_initial_to_tf=falseでは独立frame（例vio_odom）を指定してください')
            reference_position = np.zeros(3)
            reference_rotation = np.eye(3)
            if not self.p['zero_initial_pose']:
                self.A, self.b = np.eye(3), np.zeros(3)
        if self.A is None:
            self.A, self.b = yaw_alignment(position, rotation, mount[:3, :3],
                mount[:3, 3], reference_position, reference_rotation)
        if not np.isfinite(self.b).all():
            self.A = self.b = None
            raise ValueError('初回参照位置が非有限')
        self.alignment_stamp = stamp
        self.get_logger().info('VIOの位置・yawを１回だけ整列。roll/pitchは重力方向を維持')
        return True

    @staticmethod
    def _check_quaternion(q):
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError('無効なquaternion')

    def odom_callback(self, msg):
        """入力を検査し、同じ取付・初期配置でposeと全共分散を生成する。"""
        self.received += 1
        self.last_received = self.get_clock().now().nanoseconds * 1e-9
        try:
            if self.reset_required:
                self.state = 'reset_required'
                self.rejected += 1
                return
            if (msg.header.frame_id != self.p['input_frame_id']
                    or msg.child_frame_id != self.p['input_child_frame_id']):
                raise ValueError('入力frame_id/child_frame_idが設定と異なる')
            stamp = stamp_to_sec(msg.header.stamp)
            if not math.isfinite(stamp) or stamp <= 0:
                raise ValueError('入力header時刻は有限正数であることが必要')
            if self.last_stamp is not None and stamp <= self.last_stamp:
                if self.last_stamp-stamp > self.p['clock_reset_threshold_sec']:
                    self.reset_required = True
                    self.state = 'reset_required'
                    self.reason = '入力時刻が逆行。VIO epochを確認しadapter再起動が必要'
                else:
                    self.reason = '入力headerが重複または順序逆転'
                self.rejected += 1
                return
            self.last_stamp = stamp
            self._check_quaternion(msg.pose.pose.orientation)
            rotation = quat_to_rot(msg.pose.pose.orientation)
            point = msg.pose.pose.position
            position = np.array([point.x, point.y, point.z])
            if not np.isfinite(position).all():
                raise ValueError('入力位置が非有限')
            mount = self._mount()
            if mount is None:
                return
            linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
            twist = np.array([linear.x, linear.y, linear.z, angular.x, angular.y, angular.z])
            # 初回の入力共分散まで検査してから整列を確定する。不正な最初の
            # サンプルを基準として後続の正常dataまで汚染しない。
            if self.A is None:
                transform_pose(position, rotation, msg.pose.covariance, mount[:3, :3],
                    mount[:3, 3], np.eye(3), np.zeros(3), self.p['pose_covariance_orientation_frame'])
            velocity, twist_cov, _ = transform_twist(twist, msg.twist.covariance,
                mount[:3, :3], mount[:3, 3])
            if not self._initialize(position, rotation, mount, stamp):
                return
            point_out, rotation_out, pose_cov, _ = transform_pose(position, rotation,
                msg.pose.covariance, mount[:3, :3], mount[:3, 3], self.A, self.b,
                self.p['pose_covariance_orientation_frame'])
            out = Odometry()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.p['output_frame_id']
            out.child_frame_id = self.p['output_child_frame_id']
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, point_out)
            q = rot_to_quat(rotation_out)
            out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = q
            out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, velocity[:3])
            out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, velocity[3:])
            out.pose.covariance = pose_cov.reshape(-1).tolist()
            out.twist.covariance = twist_cov.reshape(-1).tolist()
            self.pub.publish(out)
            self.forwarded += 1
            self.state = 'forwarding'
            self.reason = '剛体変換済み。共分散は固定取付・初期配置を条件とする'
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.rejected += 1
            self.reason = str(exc)
            self.state = 'rejected'
            now = self.get_clock().now().nanoseconds*1e-9
            if now-self.last_warning >= 2.:
                self.last_warning = now
                self.get_logger().warn(self.reason)

    def publish_diagnostics(self):
        """１秒程度でカウンタと状態を出す。通常は多行consoleを出さない。"""
        now = self.get_clock().now()
        age = None if self.last_received is None else now.nanoseconds*1e-9-self.last_received
        state = self.state
        if age is None or age > self.p['input_timeout_sec']:
            state = 'stale'
        if self.reset_required:
            state = 'reset_required'
        status = DiagnosticStatus()
        status.name = 'vio_rigid_adapter'
        status.hardware_id = 'openvins'
        status.level = DiagnosticStatus.OK if state == 'forwarding' else DiagnosticStatus.WARN
        status.message = self.reason
        values = dict(state=state, received=self.received, forwarded=self.forwarded,
            rejected=self.rejected, waiting=self.received-self.forwarded-self.rejected, age_sec=age, alignment_stamp=self.alignment_stamp,
            mount_rotation_source=self.p['mount_rotation_source'],
            pose_covariance_orientation_frame=self.p['pose_covariance_orientation_frame'],
            covariance_condition='fixed mounting and initial alignment; unpublished correlations unavailable')
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        array = DiagnosticArray()
        array.header.stamp = now.to_msg()
        array.status = [status]
        self.diagnostics.publish(array)
        if self.p['enable_diagnostics']:
            self.get_logger().info(json.dumps(values, ensure_ascii=False))


def main(args=None):
    rclpy.init(args=args)
    node = VioOdomAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
