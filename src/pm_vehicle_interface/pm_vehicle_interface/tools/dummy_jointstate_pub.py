#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DummyJointStatePublisher(Node):
    def __init__(self):
        super().__init__("dummy_jointstate_publisher")

        # parameters
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("left_name", "left_wheel_joint")
        self.declare_parameter("right_name", "right_wheel_joint")
        self.declare_parameter("left_cycle_per_sec", 1.0)  # （設定上の）左モータ回転速度
        self.declare_parameter("right_cycle_per_sec", 1.0)  # （設定上の）右モータ回転速度

        rate = self.get_parameter("rate_hz").value
        self.left_name = self.get_parameter("left_name").value
        self.right_name = self.get_parameter("right_name").value
        self.left_cps = self.get_parameter("left_cycle_per_sec").value
        self.right_cps = self.get_parameter("right_cycle_per_sec").value

        self.pub = self.create_publisher(JointState, "/wheel_radians", 10)

        self.left_cycle = 0.0
        self.right_cycle = 0.0
        self.prev_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info("Dummy JointState publisher started")

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.prev_time = now

        self.left_cycle += self.left_cps * dt
        self.right_cycle += self.right_cps * dt

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [self.left_name, self.right_name]
        msg.position = [self.left_cycle, self.right_cycle]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DummyJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
