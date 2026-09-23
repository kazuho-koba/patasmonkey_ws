#!/usr/bin/env python3
"""Witのroll/pitchと、厳密に検証済みの最後のVIO高さをpublishする。

これは意図して慣性z積分器にはしない。VIOが停止・棄却されたら最後の正常zを保持し、
単独3D EKFで見られた無拘束の鉛直速度ドリフトを防ぐ。IMU callbackは保持値を
継続publishするため下流TFは失われないが、これは新しいz観測を意味しない。

高さcallbackは鉛直ゲートがpublishした場合だけ保持zを更新する。各IMU callbackは
Wit quaternionからroll/pitchを取り出し、保持zと組み合わせて
``/odometry/local_vertical``をpublishする。
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def rpy_from_quaternion(q):
    roll = math.atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


def quaternion_from_roll_pitch(roll, pitch):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    return sr * cp, cr * sp, -sr * sp, cr * cp


class AttitudeHeightObserverNode(Node):
    def __init__(self):
        super().__init__("attitude_height_observer_node")
        self.declare_parameter("imu_topic", "/wit/imu")
        self.declare_parameter("height_topic", "/vio/odometry/gated")
        self.declare_parameter("output_topic", "/odometry/local_vertical")
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.height_topic = str(self.get_parameter("height_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.height = 0.0
        self.height_variance = 1e6
        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.create_subscription(Odometry, self.height_topic, self.height_callback, 20)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 50)

    def height_callback(self, message):
        # このcallbackの入力は鉛直ゲートだけである。ゲートがlatchした後は、
        # localization stackを再起動するまでこの状態を意図して凍結する。
        z = message.pose.pose.position.z
        variance = message.pose.covariance[14]
        if math.isfinite(z) and math.isfinite(variance):
            self.height = z
            self.height_variance = variance

    def imu_callback(self, message):
        roll, pitch = rpy_from_quaternion(message.orientation)
        # yaw=0でquaternionを再構成する。yawは独立した水平EKFの担当であり、
        # 後段のlocal_odometry_composerが挿入する。
        qx, qy, qz, qw = quaternion_from_roll_pitch(roll, pitch)
        output = Odometry()
        output.header = message.header
        output.header.frame_id = "odom"
        output.child_frame_id = "base_link"
        # 水平位置とyawは水平EKF/composerの担当である。このnodeはroll、pitch、
        # およびゲート済み（保持中の可能性がある）zだけを担当する。
        output.pose.pose.position.z = self.height
        output.pose.pose.orientation.x = qx
        output.pose.pose.orientation.y = qy
        output.pose.pose.orientation.z = qz
        output.pose.pose.orientation.w = qw
        output.pose.covariance[14] = self.height_variance
        output.pose.covariance[21] = message.orientation_covariance[0]
        output.pose.covariance[28] = message.orientation_covariance[4]
        output.pose.covariance[35] = 1e6
        output.twist.covariance[14] = 1e6
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeHeightObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
