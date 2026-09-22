#!/usr/bin/env python3
"""Publish roll/pitch and the last strictly validated VIO height.

This is deliberately not an inertial z integrator. A stopped or rejected VIO
stream leaves z at its last valid value, avoiding the unconstrained vertical
velocity drift observed with a standalone robot_localization 3D filter.
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
        z = message.pose.pose.position.z
        variance = message.pose.covariance[14]
        if math.isfinite(z) and math.isfinite(variance):
            self.height = z
            self.height_variance = variance

    def imu_callback(self, message):
        roll, pitch = rpy_from_quaternion(message.orientation)
        qx, qy, qz, qw = quaternion_from_roll_pitch(roll, pitch)
        output = Odometry()
        output.header = message.header
        output.header.frame_id = "odom"
        output.child_frame_id = "base_link"
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
