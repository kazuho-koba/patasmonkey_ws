#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

# from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from pm_msgs.msg import MotorState


def yaw_to_quat(z_yaw: float) -> Tuple[float, float, float, float]:
    """ヨー角からクオータニオンを計算"""
    half = 0.5 * z_yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """クオータニオンからヨー角を計算"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WheelOdometryNode(Node):
    """
    Subscribe   : pm_msgs/msg/MotorState
    Publish     : nav_msgs/Odometry on /wheel/odometry
    TF          : odom -> base_link (optional)

    前提事項：
    - 左２輪、右２輪はそれぞれ同じ回転数となるスキッドステア車両
    # - JointState.positionはタイヤの累積回転数[turns](2piかけることで回転角になる)
    # - JointState velocityがある場合でも基本は無視して角度からオドメトリ計算をする
    """

    def __init__(self) -> None:
        super().__init__("wheel_odometry_node")

        # i/o topics
        self.declare_parameter("motor_state_topic", "/motor_state")
        self.declare_parameter("odom_topic", "/wheel_odometry")

        # vehicle geometry
        self.declare_parameter("wheel_radius", 0.1016)
        self.declare_parameter("tread_width", 0.36)
        self.declare_parameter("gear_ratio", 10.0)

        # frames
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)

        # initial yaw from IMU
        self.declare_parameter("use_imu_initial_yaw", False)
        self.declare_parameter("imu_topic", "/wit/imu")
        self.declare_parameter("imu_initial_timeout_sec", 5.0)

        self.motor_state_topic = str(self.get_parameter("motor_state_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.tread_width = float(self.get_parameter("tread_width").value)
        self.gear_ratio = float(self.get_parameter("gear_ratio").value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.use_imu_initial_yaw = bool(self.get_parameter("use_imu_initial_yaw").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.imu_initial_timeout_sec = float(
            self.get_parameter("imu_initial_timeout_sec").value
        )

        # error handling on vehicle geometry
        if self.wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be positive")
        if self.tread_width <= 0.0:
            raise ValueError("tread_width must be positive")
        if self.gear_ratio <= 0.0:
            raise ValueError("gear_ratio must be positive")

        # previous motor state
        self.prev_stamp: Optional[rclpy.time.Time] = None
        self.prev_left_motor_turns: Optional[float] = None  # [turns] 前フレームのモータ回転角（左）
        self.prev_right_motor_turns: Optional[float] = None  # [turns] 前フレームのモータ回転角（右）

        # integrated odometry state
        self.x = 0.0  # [m] in odom frame
        self.y = 0.0  # [m] in odom frame
        self.yaw = 0.0  # [rad] in odom frame
        # 車体初期姿勢をIMUのそれを使って補正するための関連パラメータ
        self.initial_yaw_received = False
        self.waiting_for_initial_yaw = self.use_imu_initial_yaw

        # pub/sub
        self.sub = self.create_subscription(
            MotorState,
            self.motor_state_topic,
            self.on_motor_state,
            50,
        )
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)

        # IMU情報を用いて車体の初期方位角を補正する場合、IMU情報を受け取る準備をする
        self.imu_sub = None
        if self.use_imu_initial_yaw:
            self.imu_sub = self.create_subscription(
                Imu,
                self.imu_topic,
                self.on_initial_imu,
                10,
            )

            self.imu_timeout_timer = self.create_timer(
                self.imu_initial_timeout_sec,
                self.on_imu_initial_timeout,
            )

            self.get_logger().info(
                f"Waiting for initial IMU yaw from {self.imu_topic} "
                f"for up to {self.imu_initial_timeout_sec:.1f} sec."
            )
        else:
            self.imu_timeout_timer = None

        # TFをこのノード自身が発行する場合は必要（今の想定はrobot_localizationがodom -> base_linkのTFを発行）
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            "wheel_odometry_node started: "
            f"sub={self.motor_state_topic}, "
            f"pub={self.odom_topic}, "
            f"wheel_radius={self.wheel_radius}, "
            f"tread_width={self.tread_width}, "
            f"gear_ratio={self.gear_ratio}, "
            f"publish_tf={self.publish_tf}"
        )

    def on_motor_state(self, msg: MotorState) -> None:
        # IMU向き（地磁気センサ）を使ってホイールオドメトリ側でも初期方位角を合わせる場合、逆にIMU情報を受信するまでホイールオドメトリ計算はしない。
        if self.waiting_for_initial_yaw:
            return

        # Use message stamp if provided; otherwise use current time
        if msg.stamp.sec == 0 and msg.stamp.nanosec == 0:
            now = self.get_clock().now()
        else:
            now = Time.from_msg(msg.stamp)

        # subscribeしたメッセージから左右モータ通算回転量(turns)を取得
        left_motor_turns = float(msg.left_pos_turns)
        right_motor_turns = float(msg.right_pos_turns)

        # 最初のメッセージ（前回タイヤ位置不定）のとき
        if self.prev_stamp is None:
            self.prev_stamp = now
            self.prev_left_motor_turns = left_motor_turns
            self.prev_right_motor_turns = right_motor_turns
            return

        dt = (now - self.prev_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            self.get_logger().warn(
                "Received MotorState with non-positive dt.",
                throttle_duration_sec=2.0,
            )
            return

        # モータ回転量の差分[turns]
        d_left_motor_turns = left_motor_turns - float(self.prev_left_motor_turns)
        d_right_motor_turns = right_motor_turns - float(self.prev_right_motor_turns)

        # タイヤ回転量へ変換
        d_left_wheel_turns = d_left_motor_turns / self.gear_ratio
        d_right_wheel_turns = d_right_motor_turns / self.gear_ratio

        # タイヤ移動距離
        dl = d_left_wheel_turns * 2.0 * math.pi * self.wheel_radius
        dr = d_right_wheel_turns * 2.0 * math.pi * self.wheel_radius

        # スキッドステアの動き
        ds = 0.5 * (dr + dl)
        d_yaw = (dr - dl) / self.tread_width

        # UGV位置の計算
        yaw_mid = self.yaw + 0.5 * d_yaw
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw = self._wrap_pi(self.yaw + d_yaw)

        # 速度推定
        vx = ds / dt
        wz = d_yaw / dt

        # OdometryのPublish
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz

        # 共分散のデータを仮で入れる
        odom.pose.covariance[0] = 0.05  # x
        odom.pose.covariance[7] = 0.05  # y
        odom.pose.covariance[14] = 1e6  # z
        odom.pose.covariance[21] = 1e6  # roll
        odom.pose.covariance[28] = 1e6  # pitch
        odom.pose.covariance[35] = 0.2  # yaw

        odom.twist.covariance[0] = 0.02  # vx
        odom.twist.covariance[7] = 1e6  # vy
        odom.twist.covariance[14] = 1e6  # vz
        odom.twist.covariance[21] = 1e6  # vroll
        odom.twist.covariance[28] = 1e6  # vpitch
        odom.twist.covariance[35] = 0.1  # vyaw

        self.odom_pub.publish(odom)

        # odom -> base_link のTFを発行
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        # 前回時刻情報として保存しておくパラメータの内容を更新
        self.prev_stamp = now
        self.prev_left_motor_turns = left_motor_turns
        self.prev_right_motor_turns = right_motor_turns

    def on_initial_imu(self, msg: Imu) -> None:
        if self.initial_yaw_received:
            return

        q = msg.orientation

        # orientation が無効の場合の簡易チェック
        if (
            abs(q.x) < 1e-12
            and abs(q.y) < 1e-12
            and abs(q.z) < 1e-12
            and abs(q.w) < 1e-12
        ):
            self.get_logger().warn(
                "Received IMU orientation is all zeros. Ignoring.",
                throttle_duration_sec=2.0,
            )
            return

        self.yaw = self._wrap_pi(quat_to_yaw(q.x, q.y, q.z, q.w))

        self.initial_yaw_received = True
        self.waiting_for_initial_yaw = False

        self.get_logger().info(
            f"Initial wheel odometry yaw set from IMU: {self.yaw:.4f} rad"
        )

        if self.imu_timeout_timer is not None:
            self.imu_timeout_timer.cancel()

        # 初期yaw取得後はIMU購読を解除してよい
        if self.imu_sub is not None:
            self.destroy_subscription(self.imu_sub)
            self.imu_sub = None

    def on_imu_initial_timeout(self) -> None:
        if self.initial_yaw_received:
            return

        self.waiting_for_initial_yaw = False

        self.get_logger().warn(
            "Initial IMU yaw was not received before timeout. "
            "Starting wheel odometry with yaw=0.0."
        )

        if self.imu_timeout_timer is not None:
            self.imu_timeout_timer.cancel()

        if self.imu_sub is not None:
            self.destroy_subscription(self.imu_sub)
            self.imu_sub = None

    @staticmethod
    def _index_of(names: list[str], target: str) -> Optional[int]:
        try:
            return names.index(target)
        except ValueError:
            return None

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        # 角度情報を(-pi, pi]の間に収める
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        while angle > math.pi:
            angle -= 2.0 * math.pi
        return angle


def main() -> None:
    rclpy.init()
    node = WheelOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
