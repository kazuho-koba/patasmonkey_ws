"""診断用SingleThreadedExecutorがtimer callbackを通常順序で実行しCSVを残す."""

import csv
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace

import rclpy
from rclpy.context import Context
from rclpy.node import Node

from pm_perception.executor_diagnostics import (
    EXECUTOR_FIELDS,
    SCHEDSTAT_FIELDS,
    MeasuredSingleThreadedExecutor,
    summarize_diagnostics,
)


def test_measured_executor_runs_timer_and_records_callback():
    """計測executorがcallbackを実行し、handlerのwall/CPU値を保存する."""
    context = Context()
    rclpy.init(context=context)
    node = Node("executor_probe_smoke", context=context)
    callback_calls = []
    temporary_dir = tempfile.TemporaryDirectory()
    csv_path = Path(temporary_dir.name) / "executor.csv"
    executor = MeasuredSingleThreadedExecutor(
        node, str(csv_path), flush_period_sec=0.1
    )
    executor.add_node(node)
    node.create_timer(0.005, lambda: callback_calls.append(time.monotonic()))

    try:
        deadline = time.monotonic() + 1.0
        while len(callback_calls) < 2 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(callback_calls) >= 2
    finally:
        executor.remove_node(node)
        executor.shutdown()
        executor.close()
        node.destroy_node()
        rclpy.shutdown(context=context)

    with csv_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    metadata = [row for row in rows if row["record_type"] == "metadata"]
    callbacks = [row for row in rows if row["record_type"] == "callback"]
    assert len(metadata) == 1
    assert len(callbacks) >= 2
    assert callbacks[0]["entity_type"] == "Timer"
    assert float(callbacks[0]["handler_wall_ms"]) >= 0.0
    assert callbacks[0]["handler_thread_cpu_ms"] != ""
    temporary_dir.cleanup()


def test_summary_clips_executor_events_to_schedstat_window(tmp_path, capsys):
    """schedstatのmonotonic窓外にある起動時eventをsummaryから除外する."""
    executor_path = tmp_path / "executor.csv"
    with executor_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=EXECUTOR_FIELDS)
        writer.writeheader()
        writer.writerow({
            "record_type": "metadata", "thread_id": 42, "pid": 123,
            "node_name": "depth_elevation_mapper",
        })
        for event_ns in (100, 200, 300):
            writer.writerow({
                "record_type": "callback", "monotonic_ns": event_ns,
                "thread_id": 42, "pid": 123,
                "node_name": "depth_elevation_mapper",
                "entity_type": "Subscription", "entity_name": "/oak/depth/image_raw",
                "executor_wait_ms": 0.1, "ready_to_dispatch_ms": 0.01,
                "handler_wall_ms": 1.0, "handler_thread_cpu_ms": 0.8,
                "handler_non_cpu_ms": 0.2,
            })

    schedstat_path = tmp_path / "schedstat.csv"
    with schedstat_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=SCHEDSTAT_FIELDS)
        writer.writeheader()
        writer.writerow({
            "sample_monotonic_ns": 150, "interval_wall_ns": 100,
            "pid": 123, "tid": 42, "thread_name": "depth_elevation",
            "runtime_delta_ns": "", "runqueue_wait_delta_ns": "",
            "timeslices_delta": "", "runtime_one_core_percent": "",
            "runqueue_wait_percent": "",
        })
        writer.writerow({
            "sample_monotonic_ns": 250, "interval_wall_ns": 100,
            "pid": 123, "tid": 42, "thread_name": "depth_elevation",
            "runtime_delta_ns": 80, "runqueue_wait_delta_ns": 10,
            "timeslices_delta": 2, "runtime_one_core_percent": 80.0,
            "runqueue_wait_percent": 10.0,
        })

    summarize_diagnostics(SimpleNamespace(
        executor_csv=str(executor_path), schedstat_csv=str(schedstat_path)
    ))

    output = capsys.readouterr().out
    assert "event数=1" in output
    assert "schedstat照合window" in output
    assert "/oak/depth/image_raw" in output


def test_summary_rejects_executor_and_schedstat_from_different_processes(tmp_path):
    """別processのCSVを誤結合した場合は明示的に失敗する."""
    executor_path = tmp_path / "executor.csv"
    with executor_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=EXECUTOR_FIELDS)
        writer.writeheader()
        writer.writerow({"record_type": "metadata", "thread_id": 42, "pid": 123})
        writer.writerow({
            "record_type": "callback", "monotonic_ns": 200,
            "thread_id": 42, "pid": 123, "entity_type": "Timer",
            "entity_name": "tick", "executor_wait_ms": 0.1,
            "ready_to_dispatch_ms": 0.01, "handler_wall_ms": 1.0,
            "handler_thread_cpu_ms": 0.8, "handler_non_cpu_ms": 0.2,
        })

    schedstat_path = tmp_path / "schedstat.csv"
    with schedstat_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=SCHEDSTAT_FIELDS)
        writer.writeheader()
        writer.writerow({"sample_monotonic_ns": 250, "pid": 999, "tid": 42})

    try:
        summarize_diagnostics(SimpleNamespace(
            executor_csv=str(executor_path), schedstat_csv=str(schedstat_path)
        ))
    except RuntimeError as error:
        assert "PID" in str(error)
    else:
        raise AssertionError("異なるPIDのCSVを誤って受理しました")
