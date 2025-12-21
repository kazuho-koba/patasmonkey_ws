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
    - JointState.positionはタイヤの累積角度[rad] 
    - JointState velocityがある場合でも基本は無視して角度からオドメトリ計算をする
    """

    def __init__(self) -> None:
        super().__init__("wheel_odometry_node")

        # Parameters(shared via /**: ros_parameters)
        self.declare_parameter("whl_rad", 0.1)
        self.declare_parameter("whl_sep", 0.4)
        self.declare_parameter("gear_ratio", 10.0)

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("left_joint_name", "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")
        self.declare_parameter("publish_tf", True)