"""mapperが使ったTFの保存・完全性・撮像時刻対応を検証する。"""
import pytest
from geometry_msgs.msg import TransformStamped

from pm_perception.mapper_replay_trace import MapperReplayTrace, load_mapper_trace


def test_trace_preserves_exact_transforms_and_snapshot(tmp_path):
    camera = TransformStamped()
    camera.header.frame_id = "odom"
    camera.child_frame_id = "camera"
    camera.transform.translation.z = .283456789
    camera.transform.rotation.w = 1.
    base = TransformStamped()
    base.child_frame_id = "base_link"
    base.transform.translation.z = -.173456789
    base.transform.rotation.x = .0123456789
    trace = MapperReplayTrace(tmp_path, {"resolution": .1})
    trace.fusion(1234567890123456789, "depth", [574., 574., 350., 210.], 640, 400, camera, base)
    trace.snapshot(1234567890223456789)
    trace.close()
    metadata, fusions, snapshots = load_mapper_trace(tmp_path/"mapper_trace.jsonl")
    record = fusions[1234567890123456789]
    assert record["camera_to_map"]["translation"][2] == camera.transform.translation.z
    assert record["base_to_map"]["quaternion"][0] == base.transform.rotation.x
    assert snapshots[0]["last_image_stamp_ns"] == record["image_stamp_ns"]
    assert snapshots[0]["fusion_count"] == 1
    assert metadata["parameters"]["resolution"] == .1
    with pytest.raises(FileExistsError):
        MapperReplayTrace(tmp_path, {})


def test_trace_rejects_incomplete_and_duplicate(tmp_path):
    tf = TransformStamped()
    trace = MapperReplayTrace(tmp_path, {})
    trace.fusion(123, "depth", [1., 1., 0., 0.], 2, 2, tf, tf)
    with pytest.raises(ValueError, match="未完了"):
        load_mapper_trace(tmp_path/"mapper_trace.jsonl")
    trace.fusion(123, "depth", [1., 1., 0., 0.], 2, 2, tf, tf)
    trace.close()
    with pytest.raises(ValueError, match="重複"):
        load_mapper_trace(tmp_path/"mapper_trace.jsonl")
