#!/usr/bin/env python3

from collections import deque

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPathNode(Node):
    def __init__(self):
        super().__init__("odom_to_path_node")

        self.declare_parameter("odom_topic", "/wheel/odometry")
        self.declare_parameter("path_topic", "/wheel/path")
        self.declare_parameter("max_path_length", 5000)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.max_path_length = self.get_parameter("max_path_length").value

        self.poses = deque(maxlen=self.max_path_length)

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.on_odom,
            50,
        )

        self.pub = self.create_publisher(
            Path,
            self.path_topic,
            10,
        )

        self.get_logger().info(
            f"odom_to_path_node started: {self.odom_topic} -> {self.path_topic}"
        )

    def on_odom(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.poses.append(pose)

        path = Path()
        path.header.stamp = msg.header.stamp
        path.header.frame_id = msg.header.frame_id
        path.poses = list(self.poses)

        self.pub.publish(path)


def main():
    rclpy.init()
    node = OdomToPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
