"""画像stamp対応とclock不整合guardを、既知の欠落列で確認する。"""
import struct

import pytest

from pm_evaluation.cli.analyze_mapper_depth_delivery import analyze, image_header_stamp_ns
from pm_evaluation.cli.analyze_mapper_depth_delivery import summarize_fusion_timings


def test_fusion_boundary_summary_and_old_csv():
    """retry fusionと全画像callbackを別母集団で集計し、旧CSVは欠測にする。"""
    rows = [{"record_type": "callback", "entity_name": "/depth",
             "message_stamp_ns": "1000000000", "take_wall_time_ns": "1010000000",
             "take_end_monotonic_ns": "100", "callback_start_monotonic_ns": "200"},
            {"record_type": "fusion", "header_to_fusion_ms": "60",
             "callback_to_fusion_ms": "50", "queue_wait_ms": "40",
             "fusion_processing_ms": "10", "header_to_callback_ms": "10"}]
    result = summarize_fusion_timings(rows, "/depth")
    assert result["header_to_callback_ms"]["n"] == 1
    assert result["header_to_callback_ms"]["mean"] == pytest.approx(10.0001)
    assert result["callback_to_fusion_ms"]["p95"] == 50
    assert result["queue_wait_ms"]["mean"] == 40
    assert summarize_fusion_timings(rows[:1], "/depth")["header_to_fusion_ms"]["mean"] is None


@pytest.mark.parametrize("endian,encapsulation", [("<", b"\x00\x01"), (">", b"\x00\x00")])
def test_image_header_without_deserializing_payload(endian, encapsulation):
    data = encapsulation + b"\x00\x00" + struct.pack(endian + "iI", 12, 34) + b"payload"
    assert image_header_stamp_ns(data) == 12000000034


def test_missing_images_and_rmw_clock_guard():
    rows = []
    for stamp in (100, 300):
        rows.append({"record_type": "callback", "pid": "1", "entity_name": "/depth",
                     "message_stamp_ns": str(stamp), "callback_start_monotonic_ns": str(stamp),
                     "take_end_monotonic_ns": str(stamp-1), "rmw_received_timestamp_ns": "1000",
                     "take_wall_time_ns": "900"})
    summary, missing, extra = analyze([100, 200, 300], rows, "/depth")
    assert summary["missing_at_take_count"] == 1
    assert missing[200] == 1 and not extra
    assert summary["rmw_invalid_clock_samples"] == 2
    assert summary["timings"]["received_to_take_ms"]["n"] == 0
