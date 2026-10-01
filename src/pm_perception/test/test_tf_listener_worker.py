"""TF専用threadがexact timestampの座標変換と終了処理を維持することを確認する。"""
import time
import csv
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException

from pm_perception.depth_elevation_mapper_node import _MeasuredTransformListener
from pm_perception.tf_listener_worker import TransformListenerWorker


def test_worker_preserves_timestamped_tf_and_stops(tmp_path):
    """動的TFとstaticカメラ鎖を別threadで受信し、撮像時刻の変換を得る。"""
    context = Context()
    rclpy.init(context=context)
    node = Node("tf_worker_probe", context=context)
    publisher_node = Node("tf_publisher_probe", context=context)
    buffer = Buffer()
    listener = _MeasuredTransformListener(buffer, node, diagnostics_enabled=True)
    worker = TransformListenerWorker(node, str(tmp_path / "tf.csv"))
    dynamic = publisher_node.create_publisher(TFMessage, "tf", 10)
    static = publisher_node.create_publisher(TFMessage, "tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    transform = TransformStamped()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"
    transform.header.stamp.sec = 10
    transform.transform.rotation.w = 1.0
    transform.transform.translation.x = 1.25
    camera = TransformStamped()
    camera.header.frame_id = "base_link"
    camera.child_frame_id = "camera"
    camera.transform.rotation.w = 1.0
    camera.transform.translation.z = 0.28
    result = None
    try:
        deadline = time.monotonic() + 3.0
        while result is None and time.monotonic() < deadline:
            dynamic.publish(TFMessage(transforms=[transform]))
            static.publish(TFMessage(transforms=[camera]))
            time.sleep(0.02)
            try:
                result = buffer.lookup_transform("odom", "camera", Time(seconds=10))
            except TransformException:
                pass
        assert result is not None
        assert abs(result.transform.translation.x - 1.25) < 1e-9
        assert abs(result.transform.translation.z - 0.28) < 1e-9
        assert listener.take_diagnostics()["dynamic_calls"] > 0
    finally:
        worker.close()
        assert not worker.thread.is_alive()
        rows = list(csv.DictReader((tmp_path / "tf.csv").open()))
        metadata = next(row for row in rows if row["record_type"] == "metadata")
        assert int(metadata["thread_id"]) != threading.get_native_id()
        assert all(row["thread_id"] == metadata["thread_id"] for row in rows
                   if row["record_type"] == "callback")
        listener.unregister()
        node.destroy_node()
        publisher_node.destroy_node()
        rclpy.shutdown(context=context)
