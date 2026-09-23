#!/usr/bin/env python3
"""VIOリセットを絶対位置補正として扱わず、相対速度だけをゲートする。

各``/vio/odometry``で、(1) ``vx, vy, vz``を取り出して有限値・速度・急変を
検査し、(2) 異常後は連続した健全期間を数え直し、(3) メッセージ数と時間の両方を
満たした場合だけ、元のOdometryをそのままpublishする。

EKF設定は出力Odometryのtwistだけを使う。転送messageに残るposeは、絶対位置の
補正としては決して使わない。
"""

import math

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry


class VioTwistGateNode(Node):
    def __init__(self) -> None:
        super().__init__("vio_twist_gate_node")
        self.declare_parameter("input_topic", "/vio/odometry")
        self.declare_parameter("output_topic", "/vio/odometry/twist_gated")
        self.declare_parameter("diagnostics_topic", "/vio/twist_gate/diagnostics")
        self.declare_parameter("max_linear_speed_mps", 2.5)
        self.declare_parameter("max_vertical_speed_mps", 1.0)
        self.declare_parameter("max_velocity_step_mps", 0.75)
        # 一見もっともらしい1 messageだけではVIOを再利用しない。reset直後にも
        # 有限値だが誤った相対速度が短時間出ることがある。
        self.declare_parameter("healthy_messages_required", 10)
        self.declare_parameter("healthy_duration_sec", 4.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.max_linear_speed_mps = float(self.get_parameter("max_linear_speed_mps").value)
        self.max_vertical_speed_mps = float(self.get_parameter("max_vertical_speed_mps").value)
        self.max_velocity_step_mps = float(self.get_parameter("max_velocity_step_mps").value)
        self.healthy_messages_required = int(
            self.get_parameter("healthy_messages_required").value
        )
        self.healthy_duration_sec = float(
            self.get_parameter("healthy_duration_sec").value
        )

        self.previous_velocity = None
        self.healthy_count = 0
        self.healthy_start_stamp_sec = None
        self.forwarding = False
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, self.diagnostics_topic, 10
        )
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self.callback, 50
        )
        self.publish_status("quarantine", "waiting for stable VIO twist")

    def publish_status(self, state: str, message: str) -> None:
        status = DiagnosticStatus()
        status.name = "vio_twist_gate"
        status.hardware_id = "openvins"
        status.level = DiagnosticStatus.OK if self.forwarding else DiagnosticStatus.WARN
        status.message = message
        status.values = [
            KeyValue(key="state", value=state),
            KeyValue(key="healthy_count", value=str(self.healthy_count)),
            KeyValue(key="healthy_messages_required", value=str(self.healthy_messages_required)),
            KeyValue(key="healthy_duration_sec", value=str(self.healthy_duration_sec)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_publisher.publish(array)

    def reject(self, reason: str) -> None:
        was_forwarding = self.forwarding
        self.forwarding = False
        self.healthy_count = 0
        self.healthy_start_stamp_sec = None
        # 破綻前の最後の速度と、新しいVIO epochを比較しない。次の正常messageから
        # 新しい隔離解除判定期間を開始する。
        self.previous_velocity = None
        if was_forwarding:
            self.get_logger().warn("VIO twist gate quarantined: %s" % reason)
        self.publish_status("quarantine", reason)

    def callback(self, message: Odometry) -> None:
        stamp_sec = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )
        velocity = (
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
        )
        if not all(math.isfinite(value) for value in velocity):
            self.reject("non-finite VIO velocity")
            return
        speed = math.sqrt(sum(value * value for value in velocity))
        if speed > self.max_linear_speed_mps:
            self.reject("VIO speed %.3f m/s exceeds %.3f m/s" % (
                speed, self.max_linear_speed_mps
            ))
            return
        if abs(velocity[2]) > self.max_vertical_speed_mps:
            self.reject("VIO vz %.3f m/s exceeds %.3f m/s" % (
                abs(velocity[2]), self.max_vertical_speed_mps
            ))
            return
        if self.previous_velocity is not None:
            # reset直後は有限・低速でもあり得る。差分検査で絶対速度検査だけでは
            # 見落とす不連続を捕捉する。
            step = math.sqrt(sum(
                (current - previous) ** 2
                for current, previous in zip(velocity, self.previous_velocity)
            ))
            if step > self.max_velocity_step_mps:
                self.reject("VIO velocity step %.3f m/s exceeds %.3f m/s" % (
                    step, self.max_velocity_step_mps
                ))
                return
        self.previous_velocity = velocity
        self.healthy_count += 1
        if self.healthy_start_stamp_sec is None:
            self.healthy_start_stamp_sec = stamp_sec
        healthy_duration = stamp_sec - self.healthy_start_stamp_sec
        if (not self.forwarding
                and self.healthy_count >= self.healthy_messages_required
                and healthy_duration >= self.healthy_duration_sec):
            self.forwarding = True
            self.get_logger().info(
                "VIO twist gate admitted stable velocity after %.2f s" % healthy_duration
            )
            self.publish_status(
                "forwarding", "stable VIO twist admitted after %.2f s" % healthy_duration
            )
        # 採用前のmessageは意図して消費だけし、転送しない。これによりOpenVINS
        # reset後に時間ベースの隔離期間を作る。
        if self.forwarding:
            self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VioTwistGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
