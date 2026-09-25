#!/usr/bin/env python3
"""全depth frameの対象cell診断を集計し、候補rayと姿勢変化の寄与を分ける。

入力はforensic_frame_eventsを有効にしたbag再生のCSV。2 Hzのmap snapshotに
見えなかった中間fusionも含む。前後のmin rayは必ずしも同一物理点ではないため、
counterfactualは高さ差の幾何学的な分解であり、姿勢誤差の真値判定ではない。
"""

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def rotation_from_row(row):
    """撮像時刻のcamera optical→odom quaternionを3×3回転行列へ変換する。"""
    x, y, z, w = (float(row["camera_q" + axis]) for axis in "xyzw")
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def camera_ray(event, pixel):
    """保存したpixelとdepthを、そのframeで実際に使ったKで3Dへ戻す。"""
    depth = float(pixel["axial_depth_m"])
    return np.array([
        (float(pixel["pixel_u"]) - float(event["cx"])) * depth / float(event["fx"]),
        (float(pixel["pixel_v"]) - float(event["cy"])) * depth / float(event["fy"]),
        depth,
    ])


def capture_pose(event):
    return rotation_from_row(event), np.array([
        float(event["camera_x_m"]), float(event["camera_y_m"]),
        float(event["camera_z_m"]),
    ])


def describe(values):
    values = sorted(values)
    if not values:
        return {"n": 0}
    return {"n": len(values), "min": values[0],
            "median": float(np.median(values)), "max": values[-1]}


def fitted_slope_degrees(support_rows, heights, resolution):
    """3×3 supportのXY位置を固定して平面傾斜を再計算する診断用関数。"""
    if len(support_rows) < 5:
        return float("nan")
    x0 = int(support_rows[0]["target_cell_x"])
    y0 = int(support_rows[0]["target_cell_y"])
    matrix = np.array([
        [(int(row["support_cell_x"]) - x0) * resolution,
         (int(row["support_cell_y"]) - y0) * resolution, 1.0]
        for row in support_rows
    ])
    if np.linalg.matrix_rank(matrix) < 3:
        return float("nan")
    coefficients = np.linalg.lstsq(matrix, np.asarray(heights), rcond=None)[0]
    return float(np.degrees(np.arctan(np.hypot(
        coefficients[0], coefficients[1]))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("--targets-csv", type=Path,
                        help="中心cellの集合。周囲supportを追加記録した場合の集計範囲を固定する")
    parser.add_argument("--nominal-camera-height-m", type=float, default=0.40,
                        help="mapper設定のnominal_camera_height_above_ground[m]")
    parser.add_argument("--resolution-m", type=float, default=0.05,
                        help="mapper設定のgrid resolution[m]")
    parser.add_argument("--slope-limit-deg", type=float, default=20.0,
                        help="検証設定のhazard_slope_limit_deg")
    args = parser.parse_args()
    forensic = args.result_dir / "forensic"
    with (forensic / "frame_events.csv").open(encoding="utf-8", newline="") as stream:
        events = list(csv.DictReader(stream))
    with (forensic / "frame_pixels.csv").open(encoding="utf-8", newline="") as stream:
        pixels = list(csv.DictReader(stream))
    if args.targets_csv:
        with args.targets_csv.open(encoding="utf-8", newline="") as stream:
            target_cells = {
                (int(row["odom_cell_x"]), int(row["odom_cell_y"]))
                for row in csv.DictReader(stream)
            }
    else:
        target_cells = {
            (int(row["odom_cell_x"]), int(row["odom_cell_y"])) for row in events
        }

    def key(row):
        return (int(row["odom_cell_x"]), int(row["odom_cell_y"]),
                int(row["source_stamp_ns"]))

    pixel_groups = defaultdict(list)
    for pixel in pixels:
        pixel_groups[key(pixel)].append(pixel)
    event_groups = defaultdict(list)
    for event in events:
        if key(event)[:2] in target_cells:
            event_groups[key(event)[:2]].append(event)
    event_lookup = {key(event): event for event in events}

    # 対象cellの全処理済みframeについて、保存pixel数とgridの集計数を照合する。
    # この一致を通らないデータでpose寄与を論じると、source対応の誤りになる。
    count_mismatch = []
    min_z_mismatch = []
    reprojection_errors = []
    for event in events:
        members = pixel_groups[key(event)]
        if len(members) != int(event["sample_count"]):
            count_mismatch.append(key(event))
        if members:
            saved_min = min(float(p["world_z_m"]) for p in members)
            if abs(saved_min - float(event["frame_min_world_z_m"])) > 1e-4:
                min_z_mismatch.append(key(event))
            min_pixel = min(members, key=lambda p: float(p["world_z_m"]))
            rot, translation = capture_pose(event)
            reprojected_z = float((rot @ camera_ray(event, min_pixel) + translation)[2])
            reprojection_errors.append(abs(reprojected_z - saved_min))

    replacement_rows = []
    for cell, history in sorted(event_groups.items()):
        history.sort(key=lambda row: int(row["source_stamp_ns"]))
        previous_accepted = None
        for event in history:
            mode = int(event["fusion_mode"])
            if mode == 2 and previous_accepted is not None:
                old_pixels = pixel_groups[key(previous_accepted)]
                new_pixels = pixel_groups[key(event)]
                if not old_pixels or not new_pixels:
                    continue
                old_min = min(old_pixels, key=lambda p: float(p["world_z_m"]))
                new_min = min(new_pixels, key=lambda p: float(p["world_z_m"]))
                old_ray = camera_ray(previous_accepted, old_min)
                new_ray = camera_ray(event, new_min)
                old_rot, old_t = capture_pose(previous_accepted)
                new_rot, new_t = capture_pose(event)
                old_z = float((old_rot @ old_ray + old_t)[2])
                new_z = float((new_rot @ new_ray + new_t)[2])
                old_world = old_rot @ old_ray + old_t
                new_world = new_rot @ new_ray + new_t
                # 恒等式: 新旧world-z差 = 同じ旧rayへ新poseを適用した差
                #                       + 新poseのまま新旧rayを入れ替えた差。
                translation_z = float(new_t[2] - old_t[2])
                rotation_z = float(((new_rot - old_rot) @ old_ray)[2])
                ray_z = float((new_rot @ (new_ray - old_ray))[2])
                replacement_rows.append({
                    "odom_cell_x": cell[0], "odom_cell_y": cell[1],
                    "old_stamp_ns": previous_accepted["source_stamp_ns"],
                    "new_stamp_ns": event["source_stamp_ns"],
                    "old_pixel_u": old_min["pixel_u"],
                    "old_pixel_v": old_min["pixel_v"],
                    "old_depth_m": old_min["axial_depth_m"],
                    "new_pixel_u": new_min["pixel_u"],
                    "new_pixel_v": new_min["pixel_v"],
                    "new_depth_m": new_min["axial_depth_m"],
                    "old_world_z_m": old_z, "new_world_z_m": new_z,
                    "world_xy_gap_m": float(np.linalg.norm(new_world[:2] - old_world[:2])),
                    "old_sample_count": previous_accepted["sample_count"],
                    "new_sample_count": event["sample_count"],
                    "new_frame_z_span_m": (float(event["frame_max_world_z_m"])
                                           - float(event["frame_min_world_z_m"])),
                    "candidate_world_z_delta_m": new_z - old_z,
                    "pose_translation_z_contribution_m": translation_z,
                    "pose_rotation_contribution_m": rotation_z,
                    "ray_change_contribution_m": ray_z,
                    "ground_before_m": event["ground_before_m"],
                    "ground_after_m": event["ground_after_m"],
                    "ground_update_m": (float(event["ground_after_m"])
                                        - float(event["ground_before_m"])),
                    "old_base_z_m": previous_accepted["base_z_m"],
                    "new_base_z_m": event["base_z_m"],
                })
            if mode in (1, 2, 3):
                previous_accepted = event

    out = args.result_dir / "full_frame_analysis"
    out.mkdir(parents=True, exist_ok=True)
    with (out / "lower_replace_pose_decomposition.csv").open(
            "w", encoding="utf-8", newline="") as stream:
        if replacement_rows:
            writer = csv.DictWriter(stream, fieldnames=list(replacement_rows[0]))
            writer.writeheader()
            writer.writerows(replacement_rows)

    # 2 Hzのfeature snapshotも併読し、黒への移行時点での支配cueを数える。
    # 各targetの再表示・unknownからの復帰は別onsetとして数える。
    onsets = []
    snapshot_groups = defaultdict(list)
    snapshots_by_map_cell = {}
    with (forensic / "hazard_cells.csv").open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            snapshot_groups[(int(row["odom_cell_x"]),
                             int(row["odom_cell_y"]))].append(row)
            snapshots_by_map_cell[(int(row["map_stamp_ns"]),
                                   int(row["odom_cell_x"]),
                                   int(row["odom_cell_y"]))] = row
    path_black = []
    matched_path_black = []
    with (args.result_dir / "path_samples.csv").open(
            encoding="utf-8", newline="") as stream:
        for path_row in csv.DictReader(stream):
            if path_row.get("center_hazard") != "100" or not path_row.get(
                    "lookahead_map_time_ns"):
                continue
            path_black.append(path_row)
            cell_x = math.floor(float(path_row["x_odom_m"]) / args.resolution_m)
            cell_y = math.floor(float(path_row["y_odom_m"]) / args.resolution_m)
            evidence = snapshots_by_map_cell.get((
                int(path_row["lookahead_map_time_ns"]), cell_x, cell_y))
            if evidence is not None:
                matched_path_black.append({
                    "map_stamp_ns": path_row["lookahead_map_time_ns"],
                    "odom_cell_x": cell_x, "odom_cell_y": cell_y,
                    "max_cause": evidence["max_cause"],
                    "slope_deg": evidence["slope_deg"],
                    "roughness_m": evidence["roughness_m"],
                    "step_m": evidence["step_m"],
                    "obstacle_m": evidence["obstacle_m"],
                    "fusion_mode_latest_source": evidence["fusion_mode"],
                })
    with (out / "lookahead_black_path_matches.csv").open(
            "w", encoding="utf-8", newline="") as stream:
        if matched_path_black:
            writer = csv.DictWriter(stream, fieldnames=list(matched_path_black[0]))
            writer.writeheader()
            writer.writerows(matched_path_black)
    for cell, history in snapshot_groups.items():
        history.sort(key=lambda row: int(row["map_stamp_ns"]))
        was_black = False
        for row in history:
            hazard = float(row["hazard"]) if row["hazard"] else float("nan")
            black = hazard >= 1.0
            if black and not was_black:
                onsets.append({
                    "odom_cell_x": cell[0], "odom_cell_y": cell[1],
                    "map_stamp_ns": row["map_stamp_ns"],
                    "max_cause": row["max_cause"],
                    "fusion_mode_latest_source": row["fusion_mode"],
                    "plane_support_count": row["plane_support_count"],
                    "slope_deg": row["slope_deg"],
                    "roughness_m": row["roughness_m"],
                    "step_m": row["step_m"],
                    "obstacle_m": row["obstacle_m"],
                })
            was_black = black
    with (out / "black_onsets_2hz.csv").open("w", encoding="utf-8", newline="") as stream:
        if onsets:
            writer = csv.DictWriter(stream, fieldnames=list(onsets[0]))
            writer.writeheader()
            writer.writerows(onsets)

    support_groups = defaultdict(list)
    with (forensic / "plane_support.csv").open(
            encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            support_groups[(int(row["map_stamp_ns"]),
                            int(row["target_cell_x"]),
                            int(row["target_cell_y"]))].append(row)
    slope_support_rows = []
    support_source_rows = []
    for onset in onsets:
        if onset["max_cause"] != "1":
            continue
        key_support = (int(onset["map_stamp_ns"]),
                       int(onset["odom_cell_x"]), int(onset["odom_cell_y"]))
        support = support_groups[key_support]
        first_source_index = len(support_source_rows)
        for row in support:
            source_key = (int(row["support_cell_x"]),
                          int(row["support_cell_y"]),
                          int(row["latest_source_stamp_ns"]))
            source_event = event_lookup.get(source_key)
            if source_event is None:
                continue
            source_pixels = pixel_groups[source_key]
            min_pixel = min(source_pixels, key=lambda p: float(p["world_z_m"]))
            candidate_relative = (
                float(source_event["frame_min_world_z_m"])
                + args.nominal_camera_height_m
                - float(source_event["camera_z_m"])
            )
            support_source_rows.append({
                "map_stamp_ns": onset["map_stamp_ns"],
                "target_cell_x": onset["odom_cell_x"],
                "target_cell_y": onset["odom_cell_y"],
                "support_cell_x": row["support_cell_x"],
                "support_cell_y": row["support_cell_y"],
                "source_stamp_ns": row["latest_source_stamp_ns"],
                "source_pixel_u": min_pixel["pixel_u"],
                "source_pixel_v": min_pixel["pixel_v"],
                "source_depth_m": min_pixel["axial_depth_m"],
                "source_frame_sample_count": source_event["sample_count"],
                "source_frame_z_span_m": (
                    float(source_event["frame_max_world_z_m"])
                    - float(source_event["frame_min_world_z_m"])),
                "source_fusion_mode": source_event["fusion_mode"],
                "source_relative_after_m": source_event["relative_after_m"],
                "source_candidate_relative_m": candidate_relative,
                "map_relative_elevation_m": row["relative_elevation_m"],
                "map_minus_candidate_m": (
                    float(row["relative_elevation_m"]) - candidate_relative),
            })
        # 同じ撮像frameが2個以上のsupport cellへ入っている場合、そのframe
        # 単独ですでに急な高さ差があるかを見る。各cellの融合履歴は別なので、
        # これは局所平面の因果関係を確定する値ではなく切り分け指標である。
        onset_source_rows = support_source_rows[first_source_index:]
        by_source = defaultdict(list)
        for source_row in onset_source_rows:
            by_source[source_row["source_stamp_ns"]].append(
                source_row["source_candidate_relative_m"]
            )
        shared_source_groups = [values for values in by_source.values()
                                if len(values) >= 2]
        same_frame_ranges = [max(values) - min(values)
                             for values in shared_source_groups]
        source_group_means = [float(np.mean(values))
                              for values in by_source.values()]
        # 同じsource rayを共通のcamera回転で高さ計算した感度を見る。XY割当ては
        # 固定するので仮想再投影であり、実際のmap A/Bや独立な姿勢真値ではない。
        complete_support = len(onset_source_rows) == len(support)
        if complete_support and onset_source_rows:
            reference = max(onset_source_rows,
                            key=lambda row: int(row["source_stamp_ns"]))
            reference_event = event_lookup[(
                int(reference["support_cell_x"]),
                int(reference["support_cell_y"]),
                int(reference["source_stamp_ns"]))]
            reference_rotation = rotation_from_row(reference_event)
            fixed_heights = []
            for source_row in onset_source_rows:
                source_key = (int(source_row["support_cell_x"]),
                              int(source_row["support_cell_y"]),
                              int(source_row["source_stamp_ns"]))
                source_event = event_lookup[source_key]
                source_pixel = min(pixel_groups[source_key],
                                   key=lambda p: float(p["world_z_m"]))
                fixed_heights.append(
                    float((reference_rotation @ camera_ray(
                        source_event, source_pixel))[2])
                    + args.nominal_camera_height_m
                )
            actual_heights = [row["source_candidate_relative_m"]
                              for row in onset_source_rows]
            map_heights = [float(row["map_relative_elevation_m"])
                           for row in onset_source_rows]
            map_refit = fitted_slope_degrees(
                onset_source_rows, map_heights, args.resolution_m)
            candidate_slope = fitted_slope_degrees(
                onset_source_rows, actual_heights, args.resolution_m)
            fixed_rotation_slope = fitted_slope_degrees(
                onset_source_rows, fixed_heights, args.resolution_m)
            orientation_z_max = max(abs(a - b) for a, b in zip(
                actual_heights, fixed_heights))
        else:
            map_refit = candidate_slope = fixed_rotation_slope = ""
            orientation_z_max = ""
        heights = [float(r["relative_elevation_m"]) for r in support
                   if r["relative_elevation_m"]]
        source_stamps = {int(r["latest_source_stamp_ns"])
                         for r in support if r["latest_source_stamp_ns"]}
        frame_spans = [float(r["frame_max_world_z_m"])
                       - float(r["frame_min_world_z_m"])
                       for r in support
                       if r["frame_max_world_z_m"] and r["frame_min_world_z_m"]]
        residuals = [abs(float(r["plane_residual_m"])) for r in support
                     if r["plane_residual_m"]]
        sample_counts = [int(r["frame_sample_count"]) for r in support
                         if r["frame_sample_count"]]
        slope_support_rows.append({
            **onset,
            "support_rows": len(support),
            "relative_height_range_m": max(heights) - min(heights) if heights else "",
            "unique_support_source_frames": len(source_stamps),
            "support_source_time_span_s": ((max(source_stamps) - min(source_stamps))
                                           / 1e9 if source_stamps else ""),
            "support_frame_z_span_max_m": max(frame_spans) if frame_spans else "",
            "support_plane_abs_residual_max_m": max(residuals) if residuals else "",
            "support_sample_count_min": min(sample_counts) if sample_counts else "",
            "largest_same_frame_support_group": max(
                (len(values) for values in shared_source_groups), default=0),
            "largest_same_frame_candidate_range_m": (
                max(same_frame_ranges) if same_frame_ranges else ""),
            "source_group_mean_range_m": (
                max(source_group_means) - min(source_group_means)
                if source_group_means else ""),
            "support_source_initializations": sum(
                row["source_fusion_mode"] == "1" for row in onset_source_rows),
            "support_source_rejections": sum(
                row["source_fusion_mode"] == "0" for row in onset_source_rows),
            "map_slope_refit_deg": map_refit,
            "source_candidate_slope_deg": candidate_slope,
            "fixed_camera_rotation_slope_deg": fixed_rotation_slope,
            "max_abs_camera_rotation_z_contribution_m": orientation_z_max,
        })
    with (out / "slope_black_onsets_support_2hz.csv").open(
            "w", encoding="utf-8", newline="") as stream:
        if slope_support_rows:
            writer = csv.DictWriter(stream, fieldnames=list(slope_support_rows[0]))
            writer.writeheader()
            writer.writerows(slope_support_rows)
    with (out / "slope_onset_support_sources.csv").open(
            "w", encoding="utf-8", newline="") as stream:
        if support_source_rows:
            writer = csv.DictWriter(stream, fieldnames=list(support_source_rows[0]))
            writer.writeheader()
            writer.writerows(support_source_rows)

    summary = {
        "target_cells_with_events": len(event_groups),
        "depth_cell_events": sum(len(rows) for rows in event_groups.values()),
        "unique_depth_frame_stamps_touching_targets": len({
            row["source_stamp_ns"]
            for rows in event_groups.values() for row in rows
        }),
        "all_recorded_cell_events_including_support": len(events),
        "raw_stride_pixels": sum(
            len(pixel_groups[key(row)]) for rows in event_groups.values() for row in rows),
        "fusion_modes": dict(Counter(
            row["fusion_mode"] for rows in event_groups.values() for row in rows)),
        "count_mismatch": len(count_mismatch),
        "min_z_mismatch": len(min_z_mismatch),
        "ray_reprojection_abs_error_m": describe(reprojection_errors),
        "lower_replace_count": len(replacement_rows),
        "lower_replace_ground_delta_m": describe(
            [float(row["ground_update_m"]) for row in replacement_rows]),
        "black_onsets_at_2hz": len(onsets),
        "lookahead_black_path_samples": len(path_black),
        "lookahead_black_path_exact_matches": len(matched_path_black),
        "lookahead_black_path_max_cause": dict(Counter(
            row["max_cause"] for row in matched_path_black)),
        "black_onset_max_cause": dict(Counter(row["max_cause"] for row in onsets)),
        "black_onset_latest_fusion_mode": dict(Counter(
            row["fusion_mode_latest_source"] for row in onsets)),
        "slope_black_onset_support": {
            "n": len(slope_support_rows),
            "same_source_frame": sum(
                row["unique_support_source_frames"] == 1 for row in slope_support_rows),
            "relative_height_range_m": describe([
                row["relative_height_range_m"] for row in slope_support_rows
                if row["relative_height_range_m"] != ""]),
            "support_source_time_span_s": describe([
                row["support_source_time_span_s"] for row in slope_support_rows
                if row["support_source_time_span_s"] != ""]),
            "support_frame_z_span_max_m": describe([
                row["support_frame_z_span_max_m"] for row in slope_support_rows
                if row["support_frame_z_span_max_m"] != ""]),
            "source_rows_joined_to_depth_events": len(support_source_rows),
            "source_rows_available_in_snapshot": sum(
                len(support_groups[(int(row["map_stamp_ns"]),
                                    int(row["odom_cell_x"]),
                                    int(row["odom_cell_y"]))])
                for row in slope_support_rows),
            "support_source_fusion_modes": dict(Counter(
                row["source_fusion_mode"] for row in support_source_rows)),
            "support_source_sample_count": describe([
                int(row["source_frame_sample_count"]) for row in support_source_rows]),
            "abs_map_minus_candidate_m": describe([
                abs(row["map_minus_candidate_m"]) for row in support_source_rows]),
            "support_rows_map_within_3cm_of_candidate": sum(
                abs(row["map_minus_candidate_m"]) <= 0.03
                for row in support_source_rows),
            "onsets_with_shared_source_frame": sum(
                row["largest_same_frame_support_group"] >= 2
                for row in slope_support_rows),
            "largest_same_frame_candidate_range_m": describe([
                row["largest_same_frame_candidate_range_m"]
                for row in slope_support_rows
                if row["largest_same_frame_candidate_range_m"] != ""]),
            "source_group_mean_range_m": describe([
                row["source_group_mean_range_m"] for row in slope_support_rows
                if row["source_group_mean_range_m"] != ""]),
            "onsets_with_initialized_support": sum(
                row["support_source_initializations"] > 0
                for row in slope_support_rows),
            "onsets_with_rejected_source_support": sum(
                row["support_source_rejections"] > 0
                for row in slope_support_rows),
            "complete_support_for_rotation_sensitivity": sum(
                row["fixed_camera_rotation_slope_deg"] != ""
                for row in slope_support_rows),
            "map_slope_refit_max_abs_error_deg": max((
                abs(float(row["slope_deg"]) - row["map_slope_refit_deg"])
                for row in slope_support_rows
                if row["map_slope_refit_deg"] != ""), default=0.0),
            "candidate_slope_above_limit": sum(
                row["source_candidate_slope_deg"] != ""
                and np.isfinite(row["source_candidate_slope_deg"])
                and row["source_candidate_slope_deg"] >= args.slope_limit_deg
                for row in slope_support_rows),
            "fixed_camera_rotation_slope_deg": describe([
                row["fixed_camera_rotation_slope_deg"]
                for row in slope_support_rows
                if row["fixed_camera_rotation_slope_deg"] != ""
                and np.isfinite(row["fixed_camera_rotation_slope_deg"])]),
            "onsets_fixed_rotation_below_20deg": sum(
                row["fixed_camera_rotation_slope_deg"] != ""
                and np.isfinite(row["fixed_camera_rotation_slope_deg"])
                and row["fixed_camera_rotation_slope_deg"] < args.slope_limit_deg
                for row in slope_support_rows),
            "candidate_above_limit_fixed_rotation_below_limit": sum(
                row["source_candidate_slope_deg"] != ""
                and row["fixed_camera_rotation_slope_deg"] != ""
                and row["source_candidate_slope_deg"] >= args.slope_limit_deg
                and row["fixed_camera_rotation_slope_deg"] < args.slope_limit_deg
                for row in slope_support_rows),
            "max_abs_camera_rotation_z_contribution_m": describe([
                row["max_abs_camera_rotation_z_contribution_m"]
                for row in slope_support_rows
                if row["max_abs_camera_rotation_z_contribution_m"] != ""]),
        },
        "caveat": "pose分解は異なるmin ray間の代数的分解。同じ物理点の対応は未検証。",
    }
    (out / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
