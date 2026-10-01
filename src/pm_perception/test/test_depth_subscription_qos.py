"""depth受信のreliabilityだけが切り替わり、履歴・durability等が維持される。"""
import pytest
import rclpy
import time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data
from pm_perception.depth_elevation_mapper_node import DepthElevationMapper


@pytest.mark.parametrize("value,policy,depth", [
    (None, ReliabilityPolicy.BEST_EFFORT, 3),
    ("best_effort", ReliabilityPolicy.BEST_EFFORT, 5),
    ("reliable", ReliabilityPolicy.RELIABLE, 5),
    ("reliable", ReliabilityPolicy.RELIABLE, 3),
    ("reliable", ReliabilityPolicy.RELIABLE, 2),
])
def test_depth_subscription_qos(value, policy, depth):
    """既定互換性と、実際に生成したsubscriptionのQoSを確認する。"""
    args = [] if value is None else ["--ros-args", "-p", "depth_subscription_reliability:=" + value,
                                   "-p", "depth_subscription_queue_depth:=" + str(depth)]
    rclpy.init(args=args)
    node = None
    try:
        node = DepthElevationMapper()
        profile = node.depth_subscription.qos_profile
        assert profile.reliability == policy
        assert profile.depth == depth
        for field in ("history", "durability", "deadline", "lifespan", "liveliness", "liveliness_lease_duration"):
            assert getattr(profile, field) == getattr(qos_profile_sensor_data, field)
    finally:
        if node is not None:
            node.tf_listener.unregister()
            node.destroy_node()
        rclpy.shutdown()


def test_exact_fusion_event_and_diagnostics_off():
    """空depthも処理完了として記録し、timestamp対応・単位・OFF時の無生成を確認する。"""
    rclpy.init()
    node = None
    try:
        node = DepthElevationMapper()
        message = Image()
        message.header.stamp.sec = 1
        message.height = message.width = 4
        message.encoding = "16UC1"
        message.step = 8
        message.data = bytes(32)
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        started = time.monotonic_ns()
        node.process_frame(message, (100.0, 100.0, 2.0, 2.0), transform, transform,
                           started, started, 0.0)
        event = node.diagnostic_fusion_events.popleft()
        assert event["message_stamp_ns"] == 1000000000
        assert event["fusion_end_monotonic_ns"] >= started
        assert event["callback_to_fusion_ms"] == event["fusion_processing_ms"]
        assert event["queue_wait_ms"] == 0.0
        message.header.stamp.sec = 2
        node.process_frame(message, (100.0, 100.0, 2.0, 2.0), transform, transform)
        assert not node.diagnostic_fusion_events
    finally:
        if node is not None:
            node.tf_listener.unregister()
            node.destroy_node()
        rclpy.shutdown()
