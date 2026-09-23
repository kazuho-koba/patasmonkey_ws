#!/usr/bin/env python3
"""Compose independent horizontal and attitude/height state into local odometry.

The horizontal EKF owns x/y/yaw; the attitude-height observer owns roll/pitch/z.
This node intentionally does no filtering: it is the single compatibility and
TF boundary that presents the combined state as /odometry/local.

For every horizontal EKF message, it uses the newest vertical observer message,
forms a quaternion from vertical roll/pitch plus horizontal yaw, copies the
corresponding covariance dimensions, publishes the combined Odometry, and then
optionally broadcasts the matching ``odom -> base_link`` transform.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def rpy_from_quaternion(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return roll, pitch, yaw


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class LocalOdometryComposerNode(Node):
    def __init__(self) -> None:
        super().__init__("local_odometry_composer_node")
        self.declare_parameter("horizontal_topic", "/odometry/local_horizontal")
        self.declare_parameter("vertical_topic", "/odometry/local_vertical")
        self.declare_parameter("output_topic", "/odometry/local")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_link_frame", "base_link")
        self.horizontal_topic = str(self.get_parameter("horizontal_topic").value)
        self.vertical_topic = str(self.get_parameter("vertical_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_link_frame = str(self.get_parameter("base_link_frame").value)
        self.vertical = None
        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Odometry, self.vertical_topic, self.vertical_callback, 20)
        self.create_subscription(Odometry, self.horizontal_topic, self.horizontal_callback, 50)

    def vertical_callback(self, message: Odometry) -> None:
        # Retain the most recent guarded vertical state. It may contain a held
        # z after a VIO fault, which is safer than extrapolating vertical speed.
        self.vertical = message

    def horizontal_callback(self, horizontal: Odometry) -> None:
        vertical = self.vertical
        # Horizontal messages define the output cadence and timestamp. This
        # preserves the EKF timeline even when VIO height is held or absent.
        output = Odometry()
        output.header = horizontal.header
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_link_frame
        output.pose.pose.position.x = horizontal.pose.pose.position.x
        output.pose.pose.position.y = horizontal.pose.pose.position.y
        output.pose.pose.position.z = vertical.pose.pose.position.z if vertical else 0.0
        # Take yaw only from the horizontal estimator and tilt only from the
        # attitude observer. Mixing their complete quaternions would reintroduce
        # the roll/pitch/yaw coupling this architecture is designed to avoid.
        h_roll, h_pitch, h_yaw = rpy_from_quaternion(horizontal.pose.pose.orientation)
        if vertical:
            v_roll, v_pitch, _ = rpy_from_quaternion(vertical.pose.pose.orientation)
        else:
            v_roll, v_pitch = h_roll, h_pitch
        qx, qy, qz, qw = quaternion_from_rpy(v_roll, v_pitch, h_yaw)
        output.pose.pose.orientation.x = qx
        output.pose.pose.orientation.y = qy
        output.pose.pose.orientation.z = qz
        output.pose.pose.orientation.w = qw
        output.twist.twist.linear.x = horizontal.twist.twist.linear.x
        output.twist.twist.linear.y = horizontal.twist.twist.linear.y
        output.twist.twist.linear.z = vertical.twist.twist.linear.z if vertical else 0.0
        output.twist.twist.angular.z = horizontal.twist.twist.angular.z
        if vertical:
            output.twist.twist.angular.x = vertical.twist.twist.angular.x
            output.twist.twist.angular.y = vertical.twist.twist.angular.y
        # Copy horizontal uncertainty, then replace exactly the dimensions
        # owned by the vertical observer (z, roll, pitch and related twists).
        output.pose.covariance = horizontal.pose.covariance
        output.twist.covariance = horizontal.twist.covariance
        if vertical:
            output.pose.covariance[14] = vertical.pose.covariance[14]
            output.pose.covariance[21] = vertical.pose.covariance[21]
            output.pose.covariance[28] = vertical.pose.covariance[28]
            output.twist.covariance[14] = vertical.twist.covariance[14]
            output.twist.covariance[21] = vertical.twist.covariance[21]
            output.twist.covariance[28] = vertical.twist.covariance[28]
        self.publisher.publish(output)
        if self.tf_broadcaster:
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = output.child_frame_id
            transform.transform.translation.x = output.pose.pose.position.x
            transform.transform.translation.y = output.pose.pose.position.y
            transform.transform.translation.z = output.pose.pose.position.z
            transform.transform.rotation = output.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalOdometryComposerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
