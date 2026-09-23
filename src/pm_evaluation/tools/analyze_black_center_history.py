#!/usr/bin/env python3
"""spatial-target forensic replayからblack center cellの更新履歴を要約する。

各map出力snapshotには最新source frameだけが格納される。重複するmap snapshotをsource stampで
まとめ、観測された更新mode・ground値・同一frame候補幅をcell単位で集計する。これは全10 Hz
depth updateの完全なイベントログではなく、mapperのdebug publish rateで観測できた履歴である。
"""

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


def optional_float(row, name):
    """空欄をzeroと誤認しないよう、未定義値はNoneとして返す。"""
    value = row.get(name, "")
    return float(value) if value not in (None, "") else None


def describe(values):
    """分位点計算に追加の科学計算packageを使わず、値域を要約する。"""
    values = sorted(values)
    if not values:
        return {"n": 0}

    def percentile(fraction):
        return values[int(round((len(values) - 1) * fraction))]

    return {
        "n": len(values),
        "min": values[0],
        "median": (values[(len(values) - 1) // 2] +
                   values[len(values) // 2]) * 0.5,
        "p90": percentile(0.90),
        "max": values[-1],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_dir", type=Path,
                        help="spatial_targets.csvとforensic/hazard_cells.csvのあるreplay出力先")
    args = parser.parse_args()
    target_path = args.result_dir / "spatial_targets.csv"
    cells_path = args.result_dir / "forensic" / "hazard_cells.csv"

    # target CSVに含まれる走行中心cellのみ解析し、ROI全域のforensic行は集計対象から外す。
    with target_path.open(encoding="utf-8", newline="") as stream:
        target_rows = list(csv.DictReader(stream))
    targets = {
        (int(row["odom_cell_x"]), int(row["odom_cell_y"])): int(
            row["source_sample_count"]
        )
        for row in target_rows
    }
    histories = defaultdict(list)
    with cells_path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            key = (int(row["odom_cell_x"]), int(row["odom_cell_y"]))
            if key in targets:
                histories[key].append(row)

    cell_summaries = []
    event_rows = []
    all_events = []
    for (cell_x, cell_y), source_sample_count in sorted(targets.items()):
        snapshots = sorted(
            histories.get((cell_x, cell_y), []),
            key=lambda row: int(row["map_stamp_ns"]),
        )
        # feature mapは2 Hzで保存される。隣接map snapshotが同じdepth sourceを参照する分は
        # 1観測として数え、source stampが変わった行だけevent候補にする。
        events = []
        previous_stamp = None
        for row in snapshots:
            source_stamp = row.get("latest_source_stamp_ns", "")
            if not source_stamp or source_stamp == previous_stamp:
                continue
            previous_stamp = source_stamp
            events.append(row)
        all_events.extend(events)

        modes = Counter(row.get("fusion_mode", "") for row in events)
        accepted = [row for row in events if row.get("fusion_mode") in ("2", "3")]
        relative_ground = [
            value for row in accepted
            if (value := optional_float(row, "relative_ground_after_m")) is not None
        ]
        world_ground = [
            value for row in accepted
            if (value := optional_float(row, "ground_after_fusion_m")) is not None
        ]
        absolute_base_z = [
            value for row in events
            if (value := optional_float(row, "capture_base_z_m")) is not None
        ]
        within_frame_spans = [
            high - low
            for row in events
            if (low := optional_float(row, "frame_min_world_z_m")) is not None
            and (high := optional_float(row, "frame_max_world_z_m")) is not None
        ]
        accepted_deltas = [
            after - before
            for row in accepted
            if (before := optional_float(row, "ground_before_fusion_m")) is not None
            and (after := optional_float(row, "ground_after_fusion_m")) is not None
        ]
        replace_deltas = [
            optional_float(row, "ground_after_fusion_m")
            - optional_float(row, "ground_before_fusion_m")
            for row in events if row.get("fusion_mode") == "2"
            and optional_float(row, "ground_after_fusion_m") is not None
            and optional_float(row, "ground_before_fusion_m") is not None
        ]
        hazard_values = [optional_float(row, "hazard") for row in snapshots]
        hazard_values = [value for value in hazard_values if value is not None]
        hazard_snapshots = sum(value >= 1.0 for value in hazard_values)

        if snapshots:
            first_map_ns = int(snapshots[0]["map_stamp_ns"])
            last_map_ns = int(snapshots[-1]["map_stamp_ns"])
            duration = max(0.0, (last_map_ns - first_map_ns) / 1e9)
        else:
            first_map_ns = last_map_ns = 0
            duration = 0.0

        summary = {
            "odom_cell_x": cell_x,
            "odom_cell_y": cell_y,
            "path_sample_count": source_sample_count,
            "map_snapshots": len(snapshots),
            "unique_source_stamps_seen_at_2hz": len(events),
            "history_duration_s": duration,
            "hazard_one_snapshots": hazard_snapshots,
            "fusion_mode_counts": dict(modes),
            "accepted_weighted_or_replace_ground_update_m": describe(
                [abs(value) for value in accepted_deltas]
            ),
            "lower_replace_delta_m": describe(replace_deltas),
            "same_frame_cell_z_span_m": describe(within_frame_spans),
            "accepted_relative_ground_m": describe(relative_ground),
            "accepted_world_ground_m": describe(world_ground),
            "capture_base_z_m": describe(absolute_base_z),
        }
        cell_summaries.append(summary)

        # source event CSVには、連続観測間でgroundが変わったときのsource pixel/depth/poseを残す。
        for row in events:
            event_rows.append({
                "odom_cell_x": cell_x,
                "odom_cell_y": cell_y,
                "map_stamp_ns": row["map_stamp_ns"],
                "source_stamp_ns": row.get("latest_source_stamp_ns", ""),
                "fusion_mode": row.get("fusion_mode", ""),
                "hazard": row.get("hazard", ""),
                "max_cause": row.get("max_cause", ""),
                "frame_sample_count": row.get("frame_sample_count", ""),
                "frame_min_world_z_m": row.get("frame_min_world_z_m", ""),
                "frame_max_world_z_m": row.get("frame_max_world_z_m", ""),
                "frame_min_pixel_u": row.get("frame_min_pixel_u", ""),
                "frame_min_pixel_v": row.get("frame_min_pixel_v", ""),
                "frame_min_axial_depth_m": row.get("frame_min_axial_depth_m", ""),
                "ground_before_fusion_m": row.get("ground_before_fusion_m", ""),
                "ground_after_fusion_m": row.get("ground_after_fusion_m", ""),
                "relative_ground_before_m": row.get("relative_ground_before_m", ""),
                "relative_ground_after_m": row.get("relative_ground_after_m", ""),
                "capture_base_z_m": row.get("capture_base_z_m", ""),
                "capture_roll_deg": row.get("capture_roll_deg", ""),
                "capture_pitch_deg": row.get("capture_pitch_deg", ""),
                "capture_yaw_deg": row.get("capture_yaw_deg", ""),
                "previous_ground_input_stamp_ns": row.get(
                    "previous_ground_input_stamp_ns", ""
                ),
                "previous_ground_input_relative_z_m": row.get(
                    "previous_ground_input_relative_z_m", ""
                ),
            })

    mode_totals = Counter()
    for summary in cell_summaries:
        mode_totals.update(summary["fusion_mode_counts"])
    all_replace = [
        abs(float(row["ground_after_fusion_m"]) -
            float(row["ground_before_fusion_m"]))
        for row in all_events if row.get("fusion_mode") == "2"
        and row.get("ground_before_fusion_m") not in (None, "")
        and row.get("ground_after_fusion_m") not in (None, "")
    ]
    summary = {
        "target_cells": len(targets),
        "target_cells_with_history": sum(bool(histories.get(key)) for key in targets),
        "map_snapshot_rows": sum(len(rows) for rows in histories.values()),
        "unique_source_events_sampled_at_debug_rate": len(all_events),
        "fusion_mode_counts_in_sampled_events": dict(mode_totals),
        "lower_replace_abs_delta_m": describe(all_replace),
        "cells": cell_summaries,
        "sampling_note": (
            "各map snapshotで見えたlatest sourceのみ。debug publishは通常2 Hzで、"
            "depth callbackの全更新（最大10-12 Hz）を網羅したevent logではない。"
        ),
    }

    summary_path = args.result_dir / "black_center_history_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    cell_path = args.result_dir / "black_center_cell_summary.csv"
    cell_columns = [
        "odom_cell_x", "odom_cell_y", "path_sample_count", "map_snapshots",
        "unique_source_stamps_seen_at_2hz", "history_duration_s",
        "hazard_one_snapshots", "fusion_mode_counts",
        "same_frame_cell_z_span_m", "lower_replace_delta_m",
        "accepted_relative_ground_m", "accepted_world_ground_m",
        "capture_base_z_m",
    ]
    with cell_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=cell_columns)
        writer.writeheader()
        for row in cell_summaries:
            flat = dict(row)
            for key in (
                "same_frame_cell_z_span_m", "lower_replace_delta_m",
                "accepted_relative_ground_m", "accepted_world_ground_m",
                "capture_base_z_m",
            ):
                flat[key] = json.dumps(flat[key], ensure_ascii=False)
            flat["fusion_mode_counts"] = json.dumps(
                flat["fusion_mode_counts"], ensure_ascii=False
            )
            writer.writerow({key: flat.get(key, "") for key in cell_columns})

    event_path = args.result_dir / "black_center_update_events_sampled.csv"
    event_columns = list(event_rows[0]) if event_rows else [
        "odom_cell_x", "odom_cell_y", "source_stamp_ns"
    ]
    with event_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=event_columns)
        writer.writeheader()
        writer.writerows(event_rows)

    # 全cellを一度に表示せず、ground更新の最大tailとmode数をconsoleへ要約する。
    print(json.dumps({key: value for key, value in summary.items()
                      if key != "cells"}, indent=2, ensure_ascii=False))
    print("largest relative/world ground ranges and lower replacements are in:", cell_path)
    print("sampled source events are in:", event_path)


if __name__ == "__main__":
    main()
