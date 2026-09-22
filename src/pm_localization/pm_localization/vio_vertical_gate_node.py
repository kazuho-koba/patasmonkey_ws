#!/usr/bin/env python3
"""Safety gate for the VIO height observation consumed by local EKF.

OpenVINS can reset or diverge while still publishing syntactically valid
Odometry.  This node latches closed on an implausible VIO pose/velocity so a
bad relative-height observation cannot corrupt the 3D local EKF.  It forwards
the original message unchanged while healthy; the EKF YAML selects only z.
"""

import math

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry


class VioVerticalGateNode(Node):
    def __init__(self) -> None:
        super().__init__("vio_vertical_gate_node")
        self.declare_parameter("input_topic", "/vio/odometry")
        self.declare_parameter("output_topic", "/vio/odometry/gated")
        self.declare_parameter("diagnostics_topic", "/vio/vertical_gate/diagnostics")
        # 2 m/s is the UGV's commanded upper speed. Keep a modest VIO margin;
        # the July normal bag peaked at 2.08 m/s, while August exceeded 2.5 m/s
        # immediately after its failed initialization.
        self.declare_parameter("max_linear_speed_mps", 2.5)
        self.declare_parameter("max_vertical_speed_mps", 1.0)
        # A physical terrain step cannot create a 25 cm VIO-height change in
        # one 20 Hz frame. This catches pose resets even when velocity is low.
        self.declare_parameter("max_z_step_m", 0.25)
        self.declare_parameter("max_pose_z_variance", 0.5)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.max_linear_speed_mps = float(self.get_parameter("max_linear_speed_mps").value)
        self.max_vertical_speed_mps = float(self.get_parameter("max_vertical_speed_mps").value)
        self.max_z_step_m = float(self.get_parameter("max_z_step_m").value)
        self.max_pose_z_variance = float(self.get_parameter("max_pose_z_variance").value)

        self.last_z = None
        self.latched_reason = ""
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, self.diagnostics_topic, 10
        )
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self.callback, 50
        )
        self.publish_status("healthy", "waiting for first VIO odometry")

    def publish_status(self, level_name: str, message: str) -> None:
        status = DiagnosticStatus()
        status.name = "vio_vertical_gate"
        status.hardware_id = "openvins"
        status.level = (
            DiagnosticStatus.ERROR if self.latched_reason else DiagnosticStatus.OK
        )
        status.message = message
        status.values = [
            KeyValue(key="state", value=level_name),
            KeyValue(key="latched_reason", value=self.latched_reason),
            KeyValue(key="max_linear_speed_mps", value=str(self.max_linear_speed_mps)),
            KeyValue(key="max_vertical_speed_mps", value=str(self.max_vertical_speed_mps)),
            KeyValue(key="max_z_step_m", value=str(self.max_z_step_m)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_publisher.publish(array)

    def latch(self, reason: str) -> None:
        if self.latched_reason:
            return
        self.latched_reason = reason
        self.get_logger().error("VIO vertical gate latched: %s" % reason)
        self.publish_status("latched", reason)

    def callback(self, message: Odometry) -> None:
        if self.latched_reason:
            return
        z = message.pose.pose.position.z
        vx = message.twist.twist.linear.x
        vy = message.twist.twist.linear.y
        vz = message.twist.twist.linear.z
        z_variance = message.pose.covariance[14]
        values = [z, vx, vy, vz, z_variance]
        if not all(math.isfinite(value) for value in values):
            self.latch("non-finite VIO z, velocity, or z covariance")
            return
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > self.max_linear_speed_mps:
            self.latch("VIO linear speed %.3f m/s exceeds %.3f m/s" % (
                speed, self.max_linear_speed_mps
            ))
            return
        if abs(vz) > self.max_vertical_speed_mps:
            self.latch("VIO vertical speed %.3f m/s exceeds %.3f m/s" % (
                abs(vz), self.max_vertical_speed_mps
            ))
            return
        if z_variance > self.max_pose_z_variance:
            self.latch("VIO z variance %.3f exceeds %.3f" % (
                z_variance, self.max_pose_z_variance
            ))
            return
        if self.last_z is not None and abs(z - self.last_z) > self.max_z_step_m:
            self.latch("VIO z step %.3f m exceeds %.3f m" % (
                abs(z - self.last_z), self.max_z_step_m
            ))
            return
        self.last_z = z
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VioVerticalGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
