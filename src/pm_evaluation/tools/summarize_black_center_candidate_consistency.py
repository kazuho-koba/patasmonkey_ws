#!/usr/bin/env python3
"""走行経路上の全black-center 5 cm cellでground候補の時系列一貫性を要約する。

既存forensic CSVから、hazard_cells.csvに含まれる37個の絶対odom cellを抽出し、同cellの
frame_eventsを時刻順に評価する。各frame-min候補world-zに nominal camera height - camera z
を加えたcandidate-relative-zも並べ、cameraの共通鉛直移動を差し引く前後を比較する。

rolling cellのmode 1（初期化）は独立segmentの開始として扱う。mode 2/3だけをaccepted update
として直前accepted候補と比較し、mode 0は候補として数えるが既存groundを変えないので、
accepted履歴の基準を更新しない。この集計は同じ5 cm bucketの投影値の統計であり、完全に
同一の実世界点を何度も測ったことや、候補差が物理的誤差であることを証明しない。

診断専用スクリプト。ROS node、mapper設定、再生結果は変更しない。
"""

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path


def read_csv(path):
    """UTF-8 CSVを辞書行のlistとして読み込む。今回のforensic表は数万行未満。"""
    with Path(path).open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def percentile(values, quantile):
    """numpy依存なしで昇順値のnearest-rank風percentileを返す。"""
    ordered = sorted(abs(value) for value in values)
    if not ordered:
        return None
    return ordered[int(round((len(ordered) - 1) * quantile))]


def describe(values):
    """絶対値分布をn/中央値/p90/最大/閾値超過件数で短く要約する。"""
    ordered = sorted(abs(value) for value in values)
    if not ordered:
        return {"n": 0}
    middle = len(ordered) // 2
    median = (ordered[middle] if len(ordered) % 2 else
              (ordered[middle - 1] + ordered[middle]) * 0.5)
    return {
        "n": len(ordered), "median_abs_m": median,
        "p90_abs_m": percentile(ordered, 0.90), "max_abs_m": ordered[-1],
        "abs_at_least_0p10m": sum(value >= 0.10 for value in ordered),
        "abs_at_least_0p20m": sum(value >= 0.20 for value in ordered),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("forensic_dir", type=Path,
                        help="frame_events.csvとhazard_cells.csvがあるforensic directory")
    parser.add_argument("--nominal-camera-height", type=float, default=0.40,
                        help="camera対地高さ仮定[m]。今回の7月検証値は0.40")
    parser.add_argument("--output", type=Path, required=True,
                        help="JSONとcell別CSVの出力prefix（拡張子を除く）")
    args = parser.parse_args()

    hazard_rows = read_csv(args.forensic_dir / "hazard_cells.csv")
    frame_rows = read_csv(args.forensic_dir / "frame_events.csv")
    target_cells = {
        (int(row["odom_cell_x"]), int(row["odom_cell_y"]))
        for row in hazard_rows
    }
    histories = defaultdict(list)
    for row in frame_rows:
        cell = (int(row["odom_cell_x"]), int(row["odom_cell_y"]))
        if cell in target_cells:
            histories[cell].append(row)

    modes = Counter()
    candidate_world_pairs = []
    candidate_relative_pairs = []
    fused_relative_pairs = []
    segment_world_spreads = []
    segment_relative_spreads = []
    lower_replace_events = []
    per_cell = []
    pair_rows = []

    for cell in sorted(target_cells):
        events = sorted(histories.get(cell, []),
                        key=lambda row: int(row["source_stamp_ns"]))
        segments = []
        active_segment = []
        previous_accepted = None
        for row in events:
            stamp = int(row["source_stamp_ns"])
            mode = int(row["fusion_mode"])
            modes[str(mode)] += 1
            candidate_world = float(row["frame_min_world_z_m"])
            camera_z = float(row["camera_z_m"])
            candidate_relative = (candidate_world + args.nominal_camera_height
                                  - camera_z)
            observation = {
                "source_stamp_ns": stamp, "fusion_mode": mode,
                "candidate_world_z_m": candidate_world,
                "candidate_relative_z_m": candidate_relative,
                "camera_z_m": camera_z,
                "fused_ground_before_m": float(row["ground_before_m"]),
                "fused_ground_after_m": float(row["ground_after_m"]),
                "fused_relative_before_m": float(row["relative_before_m"]),
                "fused_relative_after_m": float(row["relative_after_m"]),
                "frame_sample_count": int(row["sample_count"]),
                "frame_min_world_z_m": candidate_world,
                "frame_max_world_z_m": float(row["frame_max_world_z_m"]),
                "frame_world_z_span_m": (
                    float(row["frame_max_world_z_m"]) - candidate_world),
            }
            if mode == 1:
                # grid cellが未観測または再初期化されたら、過去segmentとのdeltaを切る。
                if active_segment:
                    segments.append(active_segment)
                active_segment = [observation]
                previous_accepted = observation
            elif mode in (2, 3):
                if previous_accepted is not None:
                    raw_delta = (candidate_world
                                 - previous_accepted["candidate_world_z_m"])
                    relative_delta = (candidate_relative
                                      - previous_accepted["candidate_relative_z_m"])
                    fused_relative_delta = (
                        observation["fused_relative_after_m"]
                        - previous_accepted["fused_relative_after_m"]
                    )
                    elapsed = (stamp - previous_accepted["source_stamp_ns"]) / 1e9
                    pair = {
                        "cell_x": cell[0], "cell_y": cell[1],
                        "from_stamp_ns": previous_accepted["source_stamp_ns"],
                        "to_stamp_ns": stamp, "interval_s": elapsed,
                        "current_fusion_mode": mode,
                        "candidate_world_z_delta_m": raw_delta,
                        "candidate_relative_z_delta_m": relative_delta,
                        "camera_z_translation_delta_m": (
                            camera_z - previous_accepted["camera_z_m"]),
                        "fused_relative_ground_delta_m": fused_relative_delta,
                        "old_ground_m": previous_accepted["fused_ground_after_m"],
                        "new_ground_before_m": observation["fused_ground_before_m"],
                        "new_ground_after_m": observation["fused_ground_after_m"],
                        "new_frame_z_span_m": observation["frame_world_z_span_m"],
                    }
                    pair_rows.append(pair)
                    candidate_world_pairs.append(raw_delta)
                    candidate_relative_pairs.append(relative_delta)
                    fused_relative_pairs.append(fused_relative_delta)
                    active_segment.append(observation)
                    if mode == 2:
                        lower_replace_events.append({
                            **pair,
                            "candidate_drop_from_old_ground_m": (
                                observation["fused_ground_before_m"] - candidate_world),
                        })
                else:
                    active_segment = [observation]
                previous_accepted = observation
            # mode 0はcandidateをgroundへ融合しない。既存ground由来のaccepted基準は維持する。

        if active_segment:
            segments.append(active_segment)
        nontrivial = [segment for segment in segments if len(segment) >= 2]
        for segment in nontrivial:
            world = [row["candidate_world_z_m"] for row in segment]
            relative = [row["candidate_relative_z_m"] for row in segment]
            segment_world_spreads.append(max(world) - min(world))
            segment_relative_spreads.append(max(relative) - min(relative))

        def spread(values):
            return max(values) - min(values) if values else None

        event_world_z = [float(row["frame_min_world_z_m"]) for row in events]
        event_relative_z = [
            float(row["frame_min_world_z_m"]) + args.nominal_camera_height
            - float(row["camera_z_m"]) for row in events
        ]
        event_fused_relative = [float(row["relative_after_m"]) for row in events]

        cell_pairs = [row for row in pair_rows
                      if row["cell_x"] == cell[0] and row["cell_y"] == cell[1]]
        per_cell.append({
            "cell_x": cell[0], "cell_y": cell[1],
            "frame_event_count": len(events),
            "mode_counts": dict(Counter(int(row["fusion_mode"]) for row in events)),
            "accepted_segments": len(segments),
            "segments_with_multiple_accepted_candidates": len(nontrivial),
            "all_event_candidate_world_z_range_m": spread(event_world_z),
            "all_event_candidate_relative_z_range_m": spread(event_relative_z),
            "all_event_fused_relative_ground_range_m": spread(event_fused_relative),
            "accepted_pair_count": len(cell_pairs),
            "max_abs_relative_candidate_step_m": max(
                (abs(row["candidate_relative_z_delta_m"]) for row in cell_pairs),
                default=None),
        })

    target_events = sum(len(rows) for rows in histories.values())
    payload = {
        "forensic_dir": str(args.forensic_dir),
        "target_center_cells_from_hazard_csv": len(target_cells),
        "center_frame_events": target_events,
        "all_frame_event_rows_including_neighbor_support": len(frame_rows),
        "noncenter_or_neighbor_frame_events": len(frame_rows) - target_events,
        "center_events_match_all_frame_event_rows": target_events == len(frame_rows),
        "nominal_camera_height_m": args.nominal_camera_height,
        "fusion_mode_counts": dict(modes),
        "accepted_update_pairs": len(pair_rows),
        "accepted_candidate_world_z_delta_m": describe(candidate_world_pairs),
        "accepted_candidate_relative_z_delta_m": describe(candidate_relative_pairs),
        "fused_relative_ground_delta_m": describe(fused_relative_pairs),
        "accepted_segment_world_z_spread_m": describe(segment_world_spreads),
        "accepted_segment_relative_z_spread_m": describe(segment_relative_spreads),
        "lower_replace_count": len(lower_replace_events),
        "lower_replace_events": lower_replace_events,
        "cells": per_cell,
        "limitations": [
            "同じabsolute odom cellは5 cm区画であり、完全に同じ実世界点の反復測定ではない",
            "candidate-relative-zは名目camera height 0.40 mを使った診断量で、測量真値ではない",
            "cell内のmin-z pixelはframeごとに別pixelになり得る",
            "この集計からodometry/depthの真値誤差を切り分けることはできない",
        ],
    }
    output_prefix = args.output
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    json_path = output_prefix.with_suffix(".json")
    csv_path = output_prefix.with_name(output_prefix.name + "_cells.csv")
    pair_path = output_prefix.with_name(output_prefix.name + "_accepted_pairs.csv")
    json_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    for path, rows, columns in (
            (csv_path, per_cell, list(per_cell[0].keys())),
            (pair_path, pair_rows, list(pair_rows[0].keys()))):
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)

    print("center cells:", len(target_cells), "events:", target_events,
          "accepted pairs:", len(pair_rows), "lower replacements:",
          len(lower_replace_events))
    print("candidate Δz:", payload["accepted_candidate_world_z_delta_m"])
    print("candidate relative Δz:", payload["accepted_candidate_relative_z_delta_m"])
    print("relative ground Δ:", payload["fused_relative_ground_delta_m"])
    print("saved:", json_path, csv_path, pair_path)


if __name__ == "__main__":
    main()
