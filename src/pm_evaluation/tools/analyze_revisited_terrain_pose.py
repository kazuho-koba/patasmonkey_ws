#!/usr/bin/env python3
"""経路の同一XY再訪を使い、odom z変化とterrain値を照合する。

経路評価CSVは走行時odom poseと、その約2 m前のhazard mapを対応付けている。
このツールは時間を空けて近いXYへ戻ったsample pairを探し、各map stamp/absolute cellの
forensic rowがあればground融合値、relative elevation、hazard cueも並べる。
"""

import argparse
import csv
import json
import math
from pathlib import Path


def load_csv(path):
    """CSVは一度だけ読む。解析対象は数百経路点と選択ROIなのでメモリは小さい。"""
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def optional_float(row, name):
    """欠測やunknown値を数値0と取り違えないよう、空欄はNoneにする。"""
    value = row.get(name, "")
    return float(value) if value not in (None, "") else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path_csv", type=Path)
    parser.add_argument("hazard_cells_csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--revisit-radius", type=float, default=0.15)
    parser.add_argument("--min-time-gap", type=float, default=30.0)
    parser.add_argument("--spatial-target-radius-cells", type=int, default=3,
                        help="最大z差revisit pairの両地点から保存するcell半径")
    args = parser.parse_args()

    path_rows = load_csv(args.path_csv)
    forensic_rows = load_csv(args.hazard_cells_csv)
    # forensic CSVはmap stamp/absolute odom cellをkeyにして、経路点のlookahead mapへ結ぶ。
    by_map_cell = {
        (int(row["map_stamp_ns"]), int(row["odom_cell_x"]), int(row["odom_cell_y"])): row
        for row in forensic_rows
    }
    poses = []
    for row in path_rows:
        if not row.get("odom_z_m"):
            continue
        poses.append({
            "stamp": int(row["passage_time_ns"]),
            "x": float(row["x_odom_m"]), "y": float(row["y_odom_m"]),
            "z": float(row["odom_z_m"]), "row": row,
        })
    poses.sort(key=lambda item: item["stamp"])

    # 近接するsample同士を重複した再訪として列挙しないよう、隣接近傍の条件は
    # 2点間距離と時間差だけに限定し、重複自体はCSV上で透明に残す。
    pairs = []
    for index, first in enumerate(poses):
        for second in poses[index + 1:]:
            dt = (second["stamp"] - first["stamp"]) / 1e9
            if dt < args.min_time_gap:
                continue
            distance = math.hypot(second["x"] - first["x"],
                                  second["y"] - first["y"])
            if distance > args.revisit_radius:
                continue
            pair = {
                "first": first, "second": second, "xy_distance_m": distance,
                "time_gap_s": dt,
                "odom_z_delta_m": second["z"] - first["z"],
            }
            for label, pose in (("first", first), ("second", second)):
                source = pose["row"]
                cell = (math.floor(pose["x"] / args.resolution),
                        math.floor(pose["y"] / args.resolution))
                map_stamp = source.get("lookahead_map_time_ns", "")
                forensic = None
                if map_stamp:
                    forensic = by_map_cell.get((int(map_stamp), cell[0], cell[1]))
                pair[label + "_map_stamp_ns"] = map_stamp
                pair[label + "_map_odom_z_m"] = optional_float(source, "map_odom_z_m")
                pair[label + "_center_hazard"] = source.get("center_hazard", "")
                pair[label + "_center_slope_deg"] = optional_float(source, "center_slope_deg")
                pair[label + "_center_roughness_m"] = optional_float(source, "center_roughness_m")
                pair[label + "_center_step_m"] = optional_float(source, "center_step_m")
                pair[label + "_center_obstacle_m"] = optional_float(source, "center_obstacle_m")
                pair[label + "_forensic_found"] = int(forensic is not None)
                for field in (
                    "ground_before_fusion_m", "ground_after_fusion_m",
                    "relative_ground_before_m", "relative_ground_after_m",
                    "fusion_mode", "capture_base_z_m", "frame_min_world_z_m",
                    "frame_max_world_z_m", "frame_min_pixel_u", "frame_min_pixel_v",
                    "frame_min_axial_depth_m", "obstacle_m", "slope_deg",
                    "roughness_m", "step_m",
                ):
                    pair[label + "_" + field] = (
                        forensic.get(field, "") if forensic else ""
                    )
            pairs.append(pair)
    pairs.sort(key=lambda pair: abs(pair["odom_z_delta_m"]), reverse=True)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    columns = [
        "xy_distance_m", "time_gap_s", "odom_z_delta_m",
        "first_stamp_ns", "first_x_m", "first_y_m", "first_odom_z_m",
        "first_map_stamp_ns", "first_map_odom_z_m", "first_center_hazard",
        "first_center_slope_deg", "first_center_roughness_m", "first_center_step_m",
        "first_center_obstacle_m", "first_forensic_found",
        "first_ground_before_fusion_m", "first_ground_after_fusion_m",
        "first_relative_ground_before_m", "first_relative_ground_after_m",
        "first_fusion_mode", "first_capture_base_z_m", "first_frame_min_world_z_m",
        "first_frame_max_world_z_m", "first_frame_min_pixel_u", "first_frame_min_pixel_v",
        "first_frame_min_axial_depth_m", "first_obstacle_m", "first_slope_deg",
        "first_roughness_m", "first_step_m",
        "second_stamp_ns", "second_x_m", "second_y_m", "second_odom_z_m",
        "second_map_stamp_ns", "second_map_odom_z_m", "second_center_hazard",
        "second_center_slope_deg", "second_center_roughness_m", "second_center_step_m",
        "second_center_obstacle_m", "second_forensic_found",
        "second_ground_before_fusion_m", "second_ground_after_fusion_m",
        "second_relative_ground_before_m", "second_relative_ground_after_m",
        "second_fusion_mode", "second_capture_base_z_m", "second_frame_min_world_z_m",
        "second_frame_max_world_z_m", "second_frame_min_pixel_u", "second_frame_min_pixel_v",
        "second_frame_min_axial_depth_m", "second_obstacle_m", "second_slope_deg",
        "second_roughness_m", "second_step_m",
    ]
    pairs_path = args.output_dir / "revisited_path_pose_pairs.csv"
    with pairs_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for pair in pairs:
            output = {
                "xy_distance_m": pair["xy_distance_m"],
                "time_gap_s": pair["time_gap_s"],
                "odom_z_delta_m": pair["odom_z_delta_m"],
            }
            for label in ("first", "second"):
                pose = pair[label]
                output[label + "_stamp_ns"] = pose["stamp"]
                output[label + "_x_m"] = pose["x"]
                output[label + "_y_m"] = pose["y"]
                output[label + "_odom_z_m"] = pose["z"]
                for column in columns:
                    prefix = label + "_"
                    if column.startswith(prefix) and column not in {
                            prefix + "stamp_ns", prefix + "x_m", prefix + "y_m",
                            prefix + "odom_z_m"}:
                        output[column] = pair.get(column, "")
            writer.writerow(output)

    z_values = [pose["z"] for pose in poses]
    summary = {
        "path_samples": len(poses),
        "path_odom_z_min_m": min(z_values) if z_values else None,
        "path_odom_z_max_m": max(z_values) if z_values else None,
        "path_odom_z_range_m": max(z_values) - min(z_values) if z_values else None,
        "revisit_radius_m": args.revisit_radius,
        "min_time_gap_s": args.min_time_gap,
        "revisit_pair_count": len(pairs),
        "pairs_with_forensic_at_both_endpoints": sum(
            pair["first_forensic_found"] and pair["second_forensic_found"]
            for pair in pairs
        ),
        "max_absolute_revisit_z_delta_m": abs(pairs[0]["odom_z_delta_m"])
        if pairs else None,
        "largest_z_delta_pair": None,
    }
    if pairs:
        largest = pairs[0]
        summary["largest_z_delta_pair"] = {
            "xy_distance_m": largest["xy_distance_m"],
            "time_gap_s": largest["time_gap_s"],
            "odom_z_delta_m": largest["odom_z_delta_m"],
            "first": {key: largest["first"][key]
                      for key in ("stamp", "x", "y", "z")},
            "second": {key: largest["second"][key]
                       for key in ("stamp", "x", "y", "z")},
            "first_relative_ground_after_m": largest["first_relative_ground_after_m"],
            "second_relative_ground_after_m": largest["second_relative_ground_after_m"],
            "first_ground_after_fusion_m": largest["first_ground_after_fusion_m"],
            "second_ground_after_fusion_m": largest["second_ground_after_fusion_m"],
            "first_max_cue_values": {
                key: largest["first_" + key] for key in (
                    "center_hazard", "center_slope_deg", "center_roughness_m",
                    "center_step_m", "center_obstacle_m")
            },
            "second_max_cue_values": {
                key: largest["second_" + key] for key in (
                    "center_hazard", "center_slope_deg", "center_roughness_m",
                    "center_step_m", "center_obstacle_m")
            },
        }
    summary_path = args.output_dir / "revisited_path_pose_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    # 次段のoffline replayではmap stampに縛らず、この再訪地点周辺のcell履歴を全時刻記録する。
    # これは小領域だけを対象とし、通常のhazard ROI全域記録よりCSV量を抑える。
    target_path = args.output_dir / "revisited_path_pose_targets.csv"
    target_cells = {}
    if pairs:
        radius = max(0, args.spatial_target_radius_cells)
        for endpoint, pose in (("first", pairs[0]["first"]),
                               ("second", pairs[0]["second"])):
            center_x = math.floor(pose["x"] / args.resolution)
            center_y = math.floor(pose["y"] / args.resolution)
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    target_cells[(center_x + dx, center_y + dy)] = {
                        "map_stamp_ns": "",
                        "odom_cell_x": center_x + dx,
                        "odom_cell_y": center_y + dy,
                        "source_endpoint": endpoint,
                        "source_center_x_m": pose["x"],
                        "source_center_y_m": pose["y"],
                    }
    with target_path.open("w", encoding="utf-8", newline="") as stream:
        columns = ("map_stamp_ns", "odom_cell_x", "odom_cell_y",
                   "source_endpoint", "source_center_x_m", "source_center_y_m")
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(target_cells.values())
    print(json.dumps(summary, indent=2))
    print("saved:", pairs_path)
    print("saved:", summary_path)
    print("saved:", target_path)


if __name__ == "__main__":
    main()
