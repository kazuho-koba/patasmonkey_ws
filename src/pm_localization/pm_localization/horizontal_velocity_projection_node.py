"""車体速度をyawだけを持つ水平車体座標へ投影する比較用node。"""
import copy
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def tilt_matrix(roll, pitch):
    """base_linkからyawを含まない水平車体座標への回転。"""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return np.array([[cp, sp * sr, sp * cr], [0., cr, -sr], [-sp, cp * sr, cp * cr]])


def project_twist(values, covariance, roll, pitch, attitude_covariance):
    """6軸twistと共分散を回転し、roll/pitch誤差の寄与を加える。

    入力値はbase_link原点の[m/s, rad/s]。線形近似は速度と姿勢誤差が
    独立という仮定を置くため、VIOとIMUの相関を正確には表さない。
    """
    rotation = tilt_matrix(roll, pitch)
    transform = np.zeros((6, 6))
    transform[:3, :3] = rotation
    transform[3:, 3:] = rotation
    values = np.asarray(values, dtype=float)
    cov = np.asarray(covariance, dtype=float).reshape((6, 6))
    result = transform @ values
    step = 1e-6
    jac = np.zeros((6, 2))
    for index, (dr, dp) in enumerate(((step, 0.), (0., step))):
        perturbed = tilt_matrix(roll + dr, pitch + dp)
        jac[:3, index] = (perturbed @ values[:3] - result[:3]) / step
        jac[3:, index] = (perturbed @ values[3:] - result[3:]) / step
    projected_cov = transform @ cov @ transform.T + jac @ attitude_covariance @ jac.T
    return result, projected_cov


class HorizontalVelocityProjectionNode(Node):
    """近傍IMU姿勢がある時のみ車輪とVIO速度を水平座標へ転送する。"""
    def __init__(self):
        super().__init__('horizontal_velocity_projection_node')
        self.declare_parameter('max_stamp_difference_sec', 0.08)
        self.declare_parameter('attitude_std_floor_deg', 2.0)
        self.max_age = float(self.get_parameter('max_stamp_difference_sec').value)
        self.attitude_variance_floor = math.radians(float(self.get_parameter('attitude_std_floor_deg').value)) ** 2
        self.samples = deque(maxlen=300)
        self.counts = {'wheel': 0, 'vio': 0, 'dropped': 0}
        self.last_imu_time = None
        self.diag_pub = self.create_publisher(DiagnosticArray, '/localization/horizontal_projection/diagnostics', 10)
        self.create_timer(1.0, self.publish_status)
        self.wheel_pub = self.create_publisher(TwistWithCovarianceStamped, '/wheel/odometry/horizontal', 20)
        self.vio_pub = self.create_publisher(TwistWithCovarianceStamped, '/vio/odometry/twist_horizontal', 20)
        self.imu_pub = self.create_publisher(Imu, '/wit/imu/horizontal', 50)
        self.create_subscription(Imu, '/wit/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/wheel/odometry', lambda m: self.on_odom(m, self.wheel_pub, True), 50)
        self.create_subscription(Odometry, '/vio/odometry/twist_gated', lambda m: self.on_odom(m, self.vio_pub), 50)

    def publish_status(self):
        """投影失敗や姿勢途絶をbagで追跡できるよう累計値を報告する。"""
        fresh = self.last_imu_time is not None and (self.get_clock().now().nanoseconds * 1e-9 - self.last_imu_time) < self.max_age * 2
        status = DiagnosticStatus()
        status.name = 'horizontal_velocity_projection'
        status.level = DiagnosticStatus.OK if fresh else DiagnosticStatus.WARN
        status.message = '姿勢を受信中' if fresh else '姿勢未受信または更新停止'
        status.values = [KeyValue(key=k, value=str(v)) for k, v in self.counts.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)

    def on_imu(self, msg):
        # 現行URDFではWitとbase_linkの軸が一致する。異なる設置では中止し、
        # TFを用いたIMU姿勢・角速度変換が必要である。
        if msg.header.frame_id != 'wit_imu_link' or msg.orientation_covariance[0] < 0:
            return
        q = msg.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-6:
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
        cov = np.asarray(msg.orientation_covariance, dtype=float).reshape((3, 3))[:2, :2]
        if not np.isfinite(cov).all() or np.any(np.diag(cov) < 0):
            return
        # ドライバの共分散だけを過信しない暫定下限。小角度近似であり、実測校正ではない。
        cov = cov.copy()
        cov[0, 0] = max(cov[0, 0], self.attitude_variance_floor)
        cov[1, 1] = max(cov[1, 1], self.attitude_variance_floor)
        t = stamp_seconds(msg.header.stamp)
        self.last_imu_time = self.get_clock().now().nanoseconds * 1e-9
        self.samples.append((t, roll, pitch, cov.copy()))
        # 水平EKFに渡す角速度も水平座標のzへ回転する。orientationと加速度は
        # EKFで無効なので意味を持たせず、回転分散のみを引き継ぐ。
        out = copy.deepcopy(msg)
        out.header.frame_id = 'base_level'
        omega = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        rotated = tilt_matrix(roll, pitch) @ omega
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = map(float, rotated)
        c = np.asarray(msg.angular_velocity_covariance, dtype=float).reshape((3, 3))
        if c[0, 0] < 0 or not np.isfinite(c).all():
            return
        full = np.zeros((6, 6)); full[3:, 3:] = c
        _, projected = project_twist([0., 0., 0., *omega], full, roll, pitch, cov)
        out.angular_velocity_covariance = projected[3:, 3:].reshape(-1).tolist()
        self.imu_pub.publish(out)

    def on_odom(self, msg, publisher, wheel=False):
        if msg.child_frame_id != 'base_link' or not self.samples:
            self.counts['dropped'] += 1
            return
        t = stamp_seconds(msg.header.stamp)
        sample = min(self.samples, key=lambda s: abs(s[0] - t))
        if abs(sample[0] - t) > self.max_age:
            self.counts['dropped'] += 1
            return
        _, roll, pitch, cov = sample
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        values = [v.x, v.y, v.z, w.x, w.y, w.z]
        if not np.isfinite(values).all() or not np.isfinite(msg.twist.covariance).all():
            self.counts['dropped'] += 1
            return
        source_cov = np.asarray(msg.twist.covariance, dtype=float).reshape((6, 6))
        if wheel:
            # 車輪から観測するのはvxのみ。vy/vzの未観測をゼロと仮定し、
            # それらの巨大な不確かさをvx観測へ混ぜない。
            values = [v.x, 0., 0., 0., 0., 0.]
            variance = source_cov[0, 0]
            source_cov = np.zeros((6, 6)); source_cov[0, 0] = variance
        if np.any(np.diag(source_cov) < 0):
            self.counts['dropped'] += 1
            return
        rotated, out_cov = project_twist(values, source_cov, roll, pitch, cov)
        out = TwistWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = 'base_level'
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, rotated[:3])
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, rotated[3:])
        out.twist.covariance = out_cov.reshape(-1).tolist()
        publisher.publish(out)
        self.counts['wheel' if wheel else 'vio'] += 1


def main(args=None):
    rclpy.init(args=args)
    node = HorizontalVelocityProjectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
