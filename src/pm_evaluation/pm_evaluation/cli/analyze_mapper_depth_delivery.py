"""MCAP画像stampとmapper take/dispatch診断を照合するオフラインCLI。

callbackに入らなかった画像を列挙する。DDSの物理配送損失、reader履歴上書き、
executor実行機会不足はこの照合だけでは分類できないため、出力名はmissing_at_takeとする。
"""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import statistics
import struct


def summarize_fusion_timings(rows, topic):
    """新CSVはfusion境界を直接集計する。旧CSVの欠測を0 msと見なさない。

    header→callbackは受信した全画像、残りは融合に成功した画像が母集団。
    monotonic差のcallback→fusionにはexact TF待ちも含まれる。
    """
    samples = {key: [] for key in (
        "header_to_callback_ms", "header_to_fusion_ms", "callback_to_fusion_ms",
        "queue_wait_ms", "fusion_processing_ms")}
    for row in rows:
        if (row.get("record_type") == "callback" and row.get("entity_name") == topic
                and all(row.get(k) for k in ("message_stamp_ns", "take_wall_time_ns",
                                            "take_end_monotonic_ns", "callback_start_monotonic_ns"))):
            samples["header_to_callback_ms"].append((
                int(row["take_wall_time_ns"]) + int(row["callback_start_monotonic_ns"])
                - int(row["take_end_monotonic_ns"]) - int(row["message_stamp_ns"]))/1e6)
        if row.get("record_type") == "fusion":
            for key in samples:
                # header→callbackは上の全受信画像から数え、成功画像を重複加算しない。
                if key != "header_to_callback_ms" and row.get(key) not in (None, ""):
                    samples[key].append(float(row[key]))
    result = {}
    for key, values in samples.items():
        ordered = sorted(values)
        result[key] = {"n": len(values), "mean": statistics.mean(values) if values else None,
                       "p50": ordered[round((len(ordered)-1)*.5)] if values else None,
                       "p95": ordered[round((len(ordered)-1)*.95)] if values else None,
                       "p99": ordered[round((len(ordered)-1)*.99)] if values else None,
                       "max": max(values) if values else None,
                       "negative_count": sum(v < 0 for v in values)}
    return result


def image_header_stamp_ns(data):
    """CDR Image先頭のHeader.stampだけ読む。data配列のdeserialize・複製をしない。"""
    if len(data) < 12 or bytes(data[:2]) not in (b"\x00\x00", b"\x00\x01"):
        raise ValueError("unsupported or truncated Image CDR encapsulation")
    endian = "<" if data[1] == 1 else ">"
    sec, nanosec = struct.unpack_from(endian + "iI", data, 4)
    if nanosec >= 1000000000:
        raise ValueError("invalid Header.stamp.nanosec")
    return sec * 1000000000 + nanosec


def interval_rate(times):
    """端点間の平均rate。観測が1件以下なら周波数を定義しない。"""
    if len(times) < 2 or max(times) == min(times):
        return None
    return (len(times) - 1) * 1e9 / (max(times) - min(times))


def read_bag_stamps(bag_path, topic):
    """対象topicだけMCAPから読む。画像本文をROS objectへ変換しない。"""
    from mcap.reader import make_reader
    path = Path(bag_path)
    files = [path] if path.is_file() else sorted(path.glob("*.mcap"))
    if not files:
        raise ValueError("MCAP file not found: %s" % path)
    stamps = []
    for file_path in files:
        with file_path.open("rb") as stream:
            for schema, channel, message in make_reader(stream).iter_messages(topics=[topic]):
                if channel.message_encoding != "cdr" or schema is None or schema.name not in (
                    "sensor_msgs/msg/Image", "sensor_msgs/Image"
                ):
                    raise ValueError("expected sensor_msgs/Image CDR channel")
                stamps.append(image_header_stamp_ns(message.data))
    return sorted(stamps)


def analyze(stamps, rows, topic):
    """共通stamp範囲で多重集合を照合し、clockが妥当なRMW時間だけ集計する。"""
    depth_rows = [row for row in rows if row.get("entity_name") == topic
                  and row.get("message_stamp_ns") and row.get("record_type") == "callback"]
    if not depth_rows:
        raise ValueError("CSV has no per-image stamps; new executor diagnostics are required")
    if len({row["pid"] for row in depth_rows}) != 1:
        raise ValueError("mixed mapper PID in executor CSV")
    taken = [int(row["message_stamp_ns"]) for row in depth_rows]
    if not stamps:
        raise ValueError("bag has no depth images")
    first, last = max(min(stamps), min(taken)), min(max(stamps), max(taken))
    if first >= last:
        raise ValueError("bag and executor stamp ranges do not overlap")
    source = [stamp for stamp in stamps if first <= stamp <= last]
    selected = [row for row in depth_rows if first <= int(row["message_stamp_ns"]) <= last]
    taken = [int(row["message_stamp_ns"]) for row in selected]
    missing = Counter(source) - Counter(taken)
    not_recorded = Counter(taken) - Counter(source)
    timing = {key: [] for key in ("take_wall_ms", "take_thread_cpu_ms")}
    timing.update({"received_to_take_ms": [], "source_to_received_ms": [], "take_to_callback_ms": []})
    invalid_clock = 0
    for row in selected:
        for key in ("take_wall_ms", "take_thread_cpu_ms"):
            if row.get(key):
                timing[key].append(float(row[key]))
        start, end = row.get("callback_start_monotonic_ns"), row.get("take_end_monotonic_ns")
        if start and end:
            timing["take_to_callback_ms"].append((int(start)-int(end))/1e6)
        received, wall = row.get("rmw_received_timestamp_ns"), row.get("take_wall_time_ns")
        if received and wall:
            delay = (int(wall)-int(received))/1e6
            # 非対応/異なるclock domainをDDS遅延として解釈しない。上限は診断guard。
            if 0 <= delay < 60000:
                timing["received_to_take_ms"].append(delay)
            else:
                invalid_clock += 1
        sent = row.get("rmw_source_timestamp_ns")
        if received and sent:
            delay = (int(received)-int(sent))/1e6
            if 0 <= delay < 60000:
                timing["source_to_received_ms"].append(delay)
    summary = {
        "stamp_range_ns": [first, last], "input_count": len(source), "take_count": len(taken),
        "input_header_rate_hz": interval_rate(source), "taken_header_rate_hz": interval_rate(taken),
        "callback_start_rate_hz": interval_rate([int(row["callback_start_monotonic_ns"])
                                                  for row in selected if row.get("callback_start_monotonic_ns")]),
        "missing_at_take_count": sum(missing.values()), "taken_not_in_bag_count": sum(not_recorded.values()),
        "rmw_invalid_clock_samples": invalid_clock,
        "timings": {key: {"n": len(values), "mean_ms": statistics.mean(values) if values else None,
                          "max_ms": max(values) if values else None} for key, values in timing.items()},
        "interpretation": "missing_at_takeはDDS配送損失・reader履歴上書き・executor待ちを単独で分類しない",
    }
    return summary, missing, not_recorded


def main():
    """解析結果と欠落stamp一覧を排他的に保存し、既存結果の上書きを防ぐ。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--executor-csv", required=True)
    parser.add_argument("--topic", default="/oak/depth/image_raw")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--schedstat-csv", help="初期化後のscheduler窓にexecutor行を限定する")
    args = parser.parse_args()
    with open(args.executor_csv, encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if args.schedstat_csv:
        with open(args.schedstat_csv, encoding="utf-8", newline="") as stream:
            samples = [int(row["sample_monotonic_ns"]) for row in csv.DictReader(stream)]
        if not samples:
            raise ValueError("empty scheduler window")
        rows = [row for row in rows if row.get("monotonic_ns") and
                min(samples) <= int(row["monotonic_ns"]) <= max(samples)]
    summary, missing, not_recorded = analyze(read_bag_stamps(args.bag, args.topic), rows, args.topic)
    summary["fusion_latency_ms"] = summarize_fusion_timings(rows, args.topic)
    if args.schedstat_csv:
        # callback率とは別に、既存fusion完了counterの増分をtimerを含む全行から合算する。
        # handler開始が窓内の行を対象にするため、境界を跨ぐhandlerの最大所要時間だけ
        # 端点の帰属に不確かさが残る。新列がない旧CSVをfusion=0と誤表示しない。
        events = [row for row in rows if row.get("record_type") == "callback"]
        window_sec = (max(samples) - min(samples)) / 1e9
        fusion_rows = [row for row in events if row.get("fused_frames_delta")]
        hazard_rows = [row for row in events if row.get("entity_name") ==
                       "DepthElevationMapper.debug_timer_callback"]
        summary["scheduler_window_sec"] = window_sec
        summary["fusion_completed_count"] = (
            sum(int(row["fused_frames_delta"]) for row in fusion_rows) if fusion_rows else None
        )
        summary["fusion_completed_rate_hz"] = (
            summary["fusion_completed_count"] / window_sec if fusion_rows else None
        )
        summary["hazard_timer_count"] = len(hazard_rows)
        summary["hazard_timer_rate_hz"] = len(hazard_rows) / window_sec
    path = Path(args.output_dir)
    path.mkdir(parents=True, exist_ok=False)
    with (path / "summary.json").open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
    with (path / "stamp_differences.csv").open("x", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["category", "header_stamp_ns", "count"])
        for category, values in (("missing_at_take", missing), ("taken_not_in_bag", not_recorded)):
            for stamp, count in sorted(values.items()):
                writer.writerow([category, stamp, count])
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
