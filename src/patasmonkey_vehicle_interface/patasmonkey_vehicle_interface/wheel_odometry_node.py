#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quat(z_yaw: float) -> Tuple[float, float, float, float]:
    """ヨー角からクオータニオンを計算"""
    half = 0.5 * z_yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class WheelOdometryNode(Node):
    """
    Subscribe   : sensor_msgs/JointState (左右のホイール角度)
    Publish     : nav_msgs/Odometry on /odom
    TF          : odom -> base_link (optional)

    前提事項：
    - 左２輪、右２輪はそれぞれ同じ回転数となるスキッドステア車両
    - JointState.positionはタイヤの累積回転数[turns](2piかけることで回転角になる)
    - JointState velocityがある場合でも基本は無視して角度からオドメトリ計算をする
    """

    def __init__(self) -> None:
        super().__init__("wheel_odometry_node")

        # Parameters(shared via /**: ros_parameters)
        self.declare_parameter("wheel_radius", 0.1)
        self.declare_parameter("tread_width", 0.36)
        self.declare_parameter("gear_ratio", 10.0)

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("left_joint_name", "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")
        self.declare_parameter("publish_tf", True)

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.tread_width = float(self.get_parameter("tread_width").value)
        self.gear_ratio = float(self.get_parameter("gear_ratio").value)

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.left_joint_name = str(self.get_parameter("left_joint_name").value)
        self.right_joint_name = str(self.get_parameter("right_joint_name").value)
        self.publish_tf = bool(self.get_parameter("publish.tf").value)

        # state
        self.prev_stamp: Optional[rclpy.time.Time] = None
        self.prev_left_pos: Optional[float] = None  # [rad] 前フレームのモータ回転角（左）
        self.prev_right_pos: Optional[float] = None  # [rad] 前フレームのモータ回転角（右）

        self.x = 0.0  # [m] in odom frame
        self.y = 0.0  # [m] in odom frame
        self.yaw = 0.0  # [rad] in odom frame

        # pub/sub
        self.sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.on_joint_state,
            50,
        )
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f"wheel_odometry_node started. Sub={self.joint_state_topic}, Pub={self.odom_topic}, "
            f"frames: {self.odom_frame}->{self.base_frame}, joints: "
            f"{self.left_joint_name}, {self.right_joint_name}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        # Find inices of left/right joints in JointState
        li = self._index_of(msg.name, self.left_joint_name)
        ri = self._index_of(msg.name, self.right_joint_name)
        if li is None or ri is None:
            # Don't spam logs too hard
            self.get_logger().warn(
                f"JointState missing required joints. Need "
                f"'{self.left_joint_name}' and '{self.right_joint_name}'. Got: {msg.name}",
                throttle_duration_sec=2.0,
            )
            return

        if len(msg.position) <= max(li, ri):
            self.get_logger().warn(
                "JointState.position is too short.", throttle_duration_sec=2.0
            )
            return

        # Use message stamp if provided; otherwise use current time
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            now = self.get_clock().now()
        else:
            now = rclpy.time.Time.from_msg(msg.header.stamp)

        # subscribeしたメッセージから左右モータ角を取得（通算回転数でデータが取れるのでradに変換）
        left_pos = float(msg.position[li]) * 2.0 * math.pi
        right_pos = float(msg.position[ri]) * 2.0 * math.pi

        # 最初のメッセージ（前回タイヤ位置不定）のとき
        if self.prev_stamp is None:
            self.prev_stamp = now
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            return

        dt = (now - self.prev_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # モータ回転角度の差分[rad]
        d_left = left_pos - float(self.prev_left_pos)
        d_right = right_pos - float(self.prev_right_pos)

        # モータ回転角度をタイヤ回転角度に変換
        d_left_wheel = d_left / self.gear_ratio
        d_right_wheel = d_right / self.gear_ratio

        # タイヤ移動距離
        dl = d_left_wheel * self.wheel_radius
        dr = d_right_wheel * self.wheel_radius

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
        # 共分散のデータもあるが今回は省略

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
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos

    @staticmethod
    def _wrap_pi(a: float) -> float:
        # 角度情報を(-pi, pi]の間に収める
        while a <= -math.pi:
            a += 2.0 * math.pi
        while a > math.pi:
            a -= 2.0 * math.pi
        return a


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
