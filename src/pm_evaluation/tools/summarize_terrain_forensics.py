#!/usr/bin/env python3
"""Forensic CSVを走行経路評価へ結合し、中心黒セルだけを抽出する。

map timestampとodom cell indexの両方をkeyにするので、別時刻や同じmap内の隣cellを誤対応
させない。全hazard ROIを読み込むのは一度だけで、近傍support CSVは該当cellの行だけ保存する。
"""

import argparse
import csv
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path


CUES = (
    ("slope", "slope_deg", 20.0),
    ("roughness", "roughness_m", 0.03),
    ("step", "step_m", 0.07),
    ("obstacle", "obstacle_m", 0.20),
)


def number(row, column):
    """CSVの空欄をNoneとして読み、数値列の型変換を一か所に集める。"""
    value = row.get(column, "")
    return float(value) if value not in (None, "") else None


def describe(values):
    """少数サンプルでも外部統計packageなしで基本分位点を返す。"""
    values = sorted(values)
    if not values:
        return {"n": 0}

    def percentile(fraction):
        index = int(round((len(values) - 1) * fraction))
        return values[index]

    return {
        "n": len(values),
        "min": values[0],
        "median": statistics.median(values),
        "p90": percentile(0.90),
        "max": values[-1],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_dir", type=Path,
                        help="path_samples.csvとforensic/を含むreplay出力先")
    parser.add_argument("--path-csv", type=Path, default=None,
                        help="別replayで得たpath_samples.csv。省略時はresult_dir内を使う")
    parser.add_argument("--target-neighborhood-cells", type=int, default=1,
                        help="各実走行中心黒cellの周囲に加える半径。姿勢/odom再生の微差を吸収する")
    args = parser.parse_args()
    forensic_dir = args.result_dir / "forensic"
    path_csv = args.path_csv or (args.result_dir / "path_samples.csv")

    # 黒だった通過地点を、診断mapの絶対odom-cellへ正確に変換してlookup表にする。
    with path_csv.open(encoding="utf-8", newline="") as stream:
        path_rows = list(csv.DictReader(stream))
    black_path_rows = [
        row for row in path_rows
        if row.get("center_hazard") == "100"
        and row.get("lookahead_map_time_ns", "")
    ]
    target_radius = max(0, args.target_neighborhood_cells)
    requested = defaultdict(list)
    target_cells = {}
    for row in black_path_rows:
        resolution = 0.05
        key = (
            int(row["lookahead_map_time_ns"]),
            math.floor(float(row["x_odom_m"]) / resolution),
            math.floor(float(row["y_odom_m"]) / resolution),
        )
        requested[key].append(row)
        for offset_y in range(-target_radius, target_radius + 1):
            for offset_x in range(-target_radius, target_radius + 1):
                target_key = (key[0], key[1] + offset_x, key[2] + offset_y)
                target_cells.setdefault(target_key, {
                    "map_stamp_ns": target_key[0],
                    "odom_cell_x": target_key[1],
                    "odom_cell_y": target_key[2],
                    "target_radius_dx": offset_x,
                    "target_radius_dy": offset_y,
                    "source_center_x_m": float(row["x_odom_m"]),
                    "source_center_y_m": float(row["y_odom_m"]),
                })
    requested_by_stamp = defaultdict(list)
    for key in requested:
        requested_by_stamp[key[0]].append(key)

    cell_matches = defaultdict(list)
    nearest_cells = {}
    cell_header = None
    cause_counts = Counter()
    fusion_modes = Counter()
    frame_sample_counts = []
    local_depth_gaps = []
    local_ground_deltas = []
    cue_masks = Counter()
    with (forensic_dir / "hazard_cells.csv").open(
            encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        cell_header = reader.fieldnames
        for row in reader:
            cause_counts[row["max_cause"]] += 1
            fusion_modes[row["fusion_mode"]] += 1
            frame_count = number(row, "frame_sample_count")
            min_z = number(row, "frame_min_world_z_m")
            max_z = number(row, "frame_max_world_z_m")
            before = number(row, "ground_before_fusion_m")
            after = number(row, "ground_after_fusion_m")
            if frame_count is not None:
                frame_sample_counts.append(frame_count)
            if min_z is not None and max_z is not None:
                local_depth_gaps.append(max_z - min_z)
            if before is not None and after is not None:
                local_ground_deltas.append(after - before)
            mask = tuple(
                name for name, column, threshold in CUES
                if number(row, column) is not None
                and number(row, column) >= threshold
            )
            cue_masks["+".join(mask) if mask else "none"] += 1
            key = (int(row["map_stamp_ns"]), int(row["odom_cell_x"]),
                   int(row["odom_cell_y"]))
            for target_key in requested_by_stamp.get(key[0], []):
                if target_key == key:
                    continue
                cell_distance = math.hypot(key[1] - target_key[1],
                                           key[2] - target_key[2])
                if (target_key not in nearest_cells
                        or cell_distance < nearest_cells[target_key][0]):
                    nearest_cells[target_key] = (cell_distance, row)
            if key in requested:
                cell_matches[key].append(row)

    # raw plane support全体を走査するが、保存対象は実走行中心黒地点の近傍だけに絞る。
    matched_keys = set(cell_matches)
    supports = defaultdict(list)
    with (forensic_dir / "plane_support.csv").open(
            encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            key = (int(row["map_stamp_ns"]), int(row["target_cell_x"]),
                   int(row["target_cell_y"]))
            if key in matched_keys:
                supports[key].append(row)

    matched_rows = []
    all_matched_cells = []
    for key, path_items in requested.items():
        candidates = cell_matches.get(key, [])
        if not candidates:
            for path_row in path_items:
                matched_rows.append({**path_row, "forensic_match": "missing"})
            continue
        # 同一map/cellが複数回出た場合は件数へ残しつつ、最新のraw観測stampの行を代表にする。
        representative = max(candidates, key=lambda item: int(item["latest_source_stamp_ns"]))
        all_matched_cells.extend([representative] * len(path_items))
        for path_row in path_items:
            matched_rows.append({**path_row, **representative, "forensic_match": "matched"})

    # cue寄与は重複を許し、どれか一つだけを原因と決めつけない。
    center_cues = Counter()
    center_modes = Counter()
    for row in all_matched_cells:
        center_modes[row["fusion_mode"]] += 1
        for name, column, threshold in CUES:
            value = number(row, column)
            if value is not None and value >= threshold:
                center_cues[name] += 1
    center_masks = Counter()
    for row in all_matched_cells:
        mask = [name for name, column, threshold in CUES
                if number(row, column) is not None and number(row, column) >= threshold]
        center_masks["+".join(mask) if mask else "none"] += 1

    matched_keys_from_rows = {
        (int(row["map_stamp_ns"]), int(row["odom_cell_x"]), int(row["odom_cell_y"]))
        for row in all_matched_cells
    }
    selected_support = [row for key in matched_keys_from_rows for row in supports.get(key, [])]
    residuals = [abs(float(row["plane_residual_m"])) for row in selected_support
                 if row.get("plane_residual_m", "")]
    # 現在セルを観測した姿勢と、その平面fitに残っている過去support姿勢との差を見る。
    # これにより深度pixelの散らばりと、複数時刻のpose重ね合わせ誤差を分けて調べやすくする。
    support_pose_delta = []
    for cell in all_matched_cells:
        key = (int(cell["map_stamp_ns"]), int(cell["odom_cell_x"]),
               int(cell["odom_cell_y"]))
        for support in supports.get(key, []):
            if not all(support.get(name, "") for name in (
                    "capture_base_x_m", "capture_base_y_m", "capture_base_z_m",
                    "capture_roll_deg", "capture_pitch_deg", "capture_yaw_deg")):
                continue
            yaw_delta = math.degrees(float(support["capture_yaw_deg"])
                                     - float(cell["capture_yaw_deg"]))
            yaw_delta = (yaw_delta + 180.0) % 360.0 - 180.0
            support_pose_delta.append({
                "dx": float(support["capture_base_x_m"]) - float(cell["capture_base_x_m"]),
                "dy": float(support["capture_base_y_m"]) - float(cell["capture_base_y_m"]),
                "dz": float(support["capture_base_z_m"]) - float(cell["capture_base_z_m"]),
                "roll": float(support["capture_roll_deg"]) - float(cell["capture_roll_deg"]),
                "pitch": float(support["capture_pitch_deg"]) - float(cell["capture_pitch_deg"]),
                "yaw": yaw_delta,
                "age": float(support["age_s"]),
            })

    def pose_stat(field):
        return describe([abs(item[field]) for item in support_pose_delta])

    plane_gradient_deg = []
    for row in all_matched_cells:
        a, b = number(row, "plane_a"), number(row, "plane_b")
        if a is not None and b is not None:
            plane_gradient_deg.append(math.degrees(math.atan(math.hypot(a, b))))
    unmatched_targets = []
    for key, path_items in requested.items():
        if key in cell_matches:
            continue
        nearest = nearest_cells.get(key)
        nearest_summary = {"nearest_cell_distance_cells": None}
        if nearest is not None:
            candidate = nearest[1]
            nearest_summary = {
                "nearest_cell_distance_cells": nearest[0],
                "nearest_forward_m": float(candidate["forward_m"]),
                "nearest_lateral_m": float(candidate["lateral_m"]),
                "nearest_hazard": float(candidate["hazard"]),
            }
        unmatched_targets.append({
            "map_stamp_ns": key[0], "odom_cell_x": key[1], "odom_cell_y": key[2],
            "x_odom_m": float(path_items[0]["x_odom_m"]),
            "y_odom_m": float(path_items[0]["y_odom_m"]), **nearest_summary,
        })

    summary = {
        "path_center_black_samples": len(black_path_rows),
        "matched_center_black_samples": len(all_matched_cells),
        "unmatched_center_black_samples": len(black_path_rows) - len(all_matched_cells),
        "forensic_hazard_cell_rows_in_front_roi": sum(cause_counts.values()),
        "forensic_max_cause_counts_in_roi": dict(cause_counts),
        "forensic_center_cue_threshold_counts": dict(center_cues),
        "forensic_center_cue_masks": dict(center_masks),
        "forensic_center_cue_value_stats": {
            name: describe([float(row[column]) for row in all_matched_cells
                            if row.get(column, "")])
            for name, column, _threshold in CUES
        },
        "forensic_center_fusion_modes": dict(center_modes),
        "forensic_fusion_modes_in_roi": dict(fusion_modes),
        "center_raw_depth_frame_sample_count": describe([
            float(row["frame_sample_count"]) for row in all_matched_cells
        ]),
        "center_raw_cell_world_z_span_m": describe([
            float(row["frame_max_world_z_m"]) - float(row["frame_min_world_z_m"])
            for row in all_matched_cells
            if row.get("frame_min_world_z_m") and row.get("frame_max_world_z_m")
        ]),
        "center_fusion_ground_delta_m": describe([
            float(row["ground_after_fusion_m"]) - float(row["ground_before_fusion_m"])
            for row in all_matched_cells
            if row.get("ground_before_fusion_m") and row.get("ground_after_fusion_m")
        ]),
        "center_residual_abs_m_across_fresh_support": describe(residuals),
        "plane_support_rows_for_center_black_cells": len(selected_support),
        "center_fitted_plane_gradient_deg": describe(plane_gradient_deg),
        "support_age_s_for_center_black_cells": describe([
            item["age"] for item in support_pose_delta
        ]),
        "support_vs_latest_capture_pose_abs_delta": {
            "x_m": pose_stat("dx"), "y_m": pose_stat("dy"),
            "z_m": pose_stat("dz"), "roll_deg": pose_stat("roll"),
            "pitch_deg": pose_stat("pitch"), "yaw_deg": pose_stat("yaw"),
        },
        "center_capture_roll_deg": describe([
            float(row["capture_roll_deg"]) for row in all_matched_cells
            if row.get("capture_roll_deg")
        ]),
        "center_capture_pitch_deg": describe([
            float(row["capture_pitch_deg"]) for row in all_matched_cells
            if row.get("capture_pitch_deg")
        ]),
        "unmatched_center_black_targets": unmatched_targets,
    }

    output = args.result_dir / "forensic_summary.json"
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    matched_path = args.result_dir / "matched_center_black.csv"
    matched_header = list(path_rows[0].keys()) + [
        name for name in (cell_header or []) if name not in path_rows[0]
    ] + ["forensic_match"]
    with matched_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=matched_header, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(matched_rows)
    matched_support_path = args.result_dir / "matched_center_black_support.csv"
    with matched_support_path.open("w", encoding="utf-8", newline="") as stream:
        if selected_support:
            writer = csv.DictWriter(stream, fieldnames=list(selected_support[0].keys()))
            writer.writeheader()
            writer.writerows(selected_support)
        else:
            stream.write("map_stamp_ns,target_cell_x,target_cell_y\n")
    targets_path = args.result_dir / "forensic_target_cells.csv"
    target_rows = list(target_cells.values())
    with targets_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(target_rows[0].keys()))
        writer.writeheader()
        writer.writerows(target_rows)

    print(json.dumps(summary, indent=2))
    print("saved:", output)
    print("saved:", matched_path)
    print("saved:", matched_support_path)
    print("saved:", targets_path)


if __name__ == "__main__":
    main()
