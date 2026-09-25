#!/usr/bin/env python3
"""黒hazard中心のplane support cell履歴をpose/ray寄与へ分解する。

対象map stampでfreshだった3x3 support cellについて、同じ絶対odom cellへ入った
過去のper-frame ground候補を読み、camera z並進・camera姿勢回転・pixel/depth rayの
寄与を隣接観測間で比較する。RGB特徴trackは使わず、mapper forensic CSVのsource pixel
とframe-min world zを結ぶため、別画素を同一点扱いする誤りを避ける。

ここでの「同じcell」は5 cm grid上の同じbucketであり、サブセル位置や地上の完全同一点を
保証しない。分解は観測された投影差の代数的な整理で、どのsensorが真値から誤ったかを
確定するものではない。
"""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


def rows(path):
    """CSVをrow dictのiteratorとして開く。大きな入力は全保持しない。"""
    with Path(path).open(encoding="utf-8", newline="") as stream:
        yield from csv.DictReader(stream)


def quaternion_matrix(x, y, z, w):
    """CSVのxyzw quaternionからcamera→odom回転行列を作る。"""
    q = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("camera quaternionのnormが0です")
    x, y, z, w = q / norm
    return np.asarray([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def camera_point(event, pixel_row):
    """pinhole Kとaxial depthから画素のoptical-frame XYZを再構成する。"""
    depth = float(pixel_row["axial_depth_m"])
    fx, fy = float(event["fx"]), float(event["fy"])
    cx, cy = float(event["cx"]), float(event["cy"])
    u, v = float(pixel_row["pixel_u"]), float(pixel_row["pixel_v"])
    return np.asarray([(u - cx) * depth / fx,
                       (v - cy) * depth / fy, depth], dtype=np.float64)


def make_observation(event, pixel_row):
    """最小world-z候補のpixelを選び、各frameの3D/poseと一緒に保持する。"""
    rotation = quaternion_matrix(*(float(event["camera_q" + axis])
                                   for axis in ("x", "y", "z", "w")))
    translation = np.asarray([float(event["camera_x_m"]),
                              float(event["camera_y_m"]),
                              float(event["camera_z_m"])], dtype=np.float64)
    point = camera_point(event, pixel_row)
    reconstructed_z = float((rotation @ point + translation)[2])
    event_min_z = float(event["frame_min_world_z_m"])
    return {
        "stamp_ns": int(event["source_stamp_ns"]),
        "fusion_mode": int(event["fusion_mode"]),
        "sample_count": int(event["sample_count"]),
        "pixel_u": int(pixel_row["pixel_u"]),
        "pixel_v": int(pixel_row["pixel_v"]),
        "depth_m": float(pixel_row["axial_depth_m"]),
        "camera_point_xyz_m": point.tolist(),
        "candidate_world_z_m": event_min_z,
        "candidate_world_xyz_m": [float(pixel_row["world_x_m"]),
                                   float(pixel_row["world_y_m"]), event_min_z],
        "reconstructed_world_z_m": reconstructed_z,
        "projection_residual_m": reconstructed_z - event_min_z,
        "candidate_relative_z_m": (
            event_min_z - translation[2] + 0.40
        ),
        "ground_before_m": float(event["ground_before_m"]),
        "ground_after_m": float(event["ground_after_m"]),
        "relative_before_m": float(event["relative_before_m"]),
        "relative_after_m": float(event["relative_after_m"]),
        "camera_translation_xyz_m": translation.tolist(),
        "camera_rotation_matrix": rotation.tolist(),
        "base_z_m": float(event["base_z_m"]),
        "base_roll_deg": math.degrees(float(event["base_roll_rad"])),
        "base_pitch_deg": math.degrees(float(event["base_pitch_rad"])),
        "base_yaw_deg": math.degrees(float(event["base_yaw_rad"])),
    }


def decompose_pair(previous, current):
    """連続観測のworld-z差をtranslation、rotation、ray項へ厳密分解する。

    R1 p1 - R0 p0 = Rbar (p1-p0) + (R1-R0) pbar
    と対称分解する。Rbarは2回転行列の平均で、物理的な姿勢ではなく、交差項を
    どちらか一方へ恣意的に割り当てないための代数上の分解である。
    """
    r0 = np.asarray(previous["camera_rotation_matrix"], dtype=np.float64)
    r1 = np.asarray(current["camera_rotation_matrix"], dtype=np.float64)
    p0 = np.asarray(previous["camera_point_xyz_m"], dtype=np.float64)
    p1 = np.asarray(current["camera_point_xyz_m"], dtype=np.float64)
    t0 = np.asarray(previous["camera_translation_xyz_m"], dtype=np.float64)
    t1 = np.asarray(current["camera_translation_xyz_m"], dtype=np.float64)
    mean_rotation = 0.5 * (r0 + r1)
    mean_point = 0.5 * (p0 + p1)
    translation_dz = float(t1[2] - t0[2])
    rotation_dz = float(((r1 - r0) @ mean_point)[2])
    ray_dz = float((mean_rotation @ (p1 - p0))[2])
    observed_dz = float(current["candidate_world_z_m"]
                        - previous["candidate_world_z_m"])
    xy0 = np.asarray(previous["candidate_world_xyz_m"][:2], dtype=np.float64)
    xy1 = np.asarray(current["candidate_world_xyz_m"][:2], dtype=np.float64)
    return {
        "from_stamp_ns": previous["stamp_ns"],
        "to_stamp_ns": current["stamp_ns"],
        "interval_s": (current["stamp_ns"] - previous["stamp_ns"]) / 1e9,
        "from_pixel_uv": [previous["pixel_u"], previous["pixel_v"]],
        "to_pixel_uv": [current["pixel_u"], current["pixel_v"]],
        "from_depth_m": previous["depth_m"], "to_depth_m": current["depth_m"],
        "world_z_candidate_delta_m": observed_dz,
        "projected_xy_delta_m": float(np.linalg.norm(xy1 - xy0)),
        "camera_translation_z_delta_m": translation_dz,
        "camera_rotation_contribution_m": rotation_dz,
        "pixel_and_depth_ray_contribution_m": ray_dz,
        "component_sum_residual_m": observed_dz - translation_dz - rotation_dz - ray_dz,
        "relative_ground_fusion_delta_m": (
            current["relative_after_m"] - previous["relative_after_m"]
        ),
        "previous_fused_ground_m": previous["ground_after_m"],
        "current_ground_before_m": current["ground_before_m"],
        "current_ground_after_m": current["ground_after_m"],
        "current_fusion_mode": current["fusion_mode"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("forensic_dir", type=Path,
                        help="frame_events.csv / frame_pixels.csv / plane_support.csv")
    parser.add_argument("color_projection_summary", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    summary = json.loads(args.color_projection_summary.read_text(encoding="utf-8"))
    # RGB追跡対象を一部だけ選ばず、summaryにあるblack-center全地点を処理する。
    # 代表例の外にあるfusion履歴も同じ条件で比較できるようにする。
    targets = {int(item["index"]): item for item in summary["targets"]}
    if not targets:
        raise RuntimeError("color projection summaryにtargetがありません")

    # まず各対象black centerの3x3 plane supportと、その9セルの絶対座標を集める。
    target_cells = {}
    support_snapshot = {}
    for index, target in targets.items():
        key = (int(target["map_stamp_ns"]), int(target["odom_cell_x"]),
               int(target["odom_cell_y"]))
        support_snapshot[index] = []
        target_cells[index] = set()
        for row in rows(args.forensic_dir / "plane_support.csv"):
            row_key = (int(row["map_stamp_ns"]), int(row["target_cell_x"]),
                       int(row["target_cell_y"]))
            if row_key == key:
                cell = (int(row["support_cell_x"]), int(row["support_cell_y"]))
                target_cells[index].add(cell)
                support_snapshot[index].append({
                    "support_cell_x": cell[0], "support_cell_y": cell[1],
                    "relative_elevation_m": float(row["relative_elevation_m"]),
                    "age_s": float(row["age_s"]),
                    "plane_residual_m": float(row["plane_residual_m"]),
                    "latest_source_stamp_ns": int(row["latest_source_stamp_ns"]),
                })
        # rolling window端やfresh support不足で、snapshotによってplane support数は
        # 9未満になり得る。summary側の実測数と照合し、欠けたcellを勝手に補わない。
        expected_support_count = int(target.get("support_count", len(target_cells[index])))
        if len(target_cells[index]) != expected_support_count:
            raise RuntimeError(
                "target {} plane support数がsummaryと不一致: {} != {}".format(
                    index, len(target_cells[index]), expected_support_count))

    # 対象3x3 cellのmap stampまでの観測履歴だけ読む。
    event_by_target = {index: defaultdict(list) for index in targets}
    wanted_event_keys = set()
    for row in rows(args.forensic_dir / "frame_events.csv"):
        stamp = int(row["source_stamp_ns"])
        cell = (int(row["odom_cell_x"]), int(row["odom_cell_y"]))
        for index, target in targets.items():
            if (cell in target_cells[index]
                    and stamp <= int(target["map_stamp_ns"])):
                key = (stamp, cell[0], cell[1])
                wanted_event_keys.add(key)
                event_by_target[index][cell].append(row)

    # frame_pixelsにはsampleされた全pixelがある。対象eventのcell内で最小world-zに
    # 最も近いpixelを探し、mapperがground candidateに使ったrayの代表とする。
    best_pixel = {}
    for row in rows(args.forensic_dir / "frame_pixels.csv"):
        key = (int(row["source_stamp_ns"]), int(row["odom_cell_x"]),
               int(row["odom_cell_y"]))
        if key not in wanted_event_keys:
            continue
        z = float(row["world_z_m"])
        current = best_pixel.get(key)
        if current is None or z < current[0]:
            best_pixel[key] = (z, row)

    target_results = []
    missing_candidates = []
    for index, target in targets.items():
        cell_results = []
        for cell in sorted(target_cells[index]):
            history = []
            for event in sorted(event_by_target[index][cell],
                                key=lambda item: int(item["source_stamp_ns"])):
                key = (int(event["source_stamp_ns"]), cell[0], cell[1])
                pixel_match = best_pixel.get(key)
                if pixel_match is None:
                    missing_candidates.append({"target_index": index,
                                               "cell": list(cell), "stamp_ns": key[0]})
                    continue
                observation = make_observation(event, pixel_match[1])
                # 棄却(mode 0)も一覧には残し、accepted履歴は別欄で追えるようにする。
                history.append(observation)
            accepted = [item for item in history if item["fusion_mode"] in (1, 2, 3)]
            # mode 1は未観測/rolling slot再利用後の初期化で、前のaccepted値を
            # 引き継がない。従ってそこを時間系列の境界としてpairを切る。
            pair_deltas = []
            previous = None
            for item in history:
                if item["fusion_mode"] == 1:
                    previous = item
                elif item["fusion_mode"] in (2, 3):
                    if previous is not None:
                        pair_deltas.append(decompose_pair(previous, item))
                    previous = item
            snapshot = next((row for row in support_snapshot[index]
                             if (row["support_cell_x"], row["support_cell_y"]) == cell), None)
            latest = next((item for item in reversed(accepted)
                           if item["stamp_ns"] == snapshot["latest_source_stamp_ns"]), None)
            cell_results.append({
                "cell": list(cell), "support_snapshot": snapshot,
                "latest_accepted_candidate": latest,
                "observations_before_target_map": history,
                "accepted_observation_count": len(accepted),
                "rejected_observation_count": sum(
                    item["fusion_mode"] == 0 for item in history),
                "consecutive_accepted_observation_deltas": pair_deltas,
            })
        target_results.append({
            "target_index": index, "map_stamp_ns": int(target["map_stamp_ns"]),
            "hazard_cell": [int(target["odom_cell_x"]), int(target["odom_cell_y"])],
            "max_cause": int(target["max_cause"]),
            "slope_deg": target.get("slope_deg"),
            "step_m": target.get("step_m"),
            "support_height_range_m": target.get("support_map_height_range_m"),
            "support_cell_count": len(target_cells[index]),
            "support_cells": cell_results,
        })

    output = {
        "input_forensic_dir": str(args.forensic_dir),
        "decomposition": (
            "delta world z = delta camera translation z + symmetric camera rotation term "
            "+ symmetric pixel/depth-ray term; the latter is not sensor noise alone"
        ),
        "ground_candidate_selection": (
            "within each source stamp and odom cell, choose the sampled pixel with minimum "
            "world_z; this matches the mapper's per-cell min-z ground candidate"
        ),
        "targets": target_results,
        "missing_pixel_candidates": missing_candidates,
        "limitations": [
            "同じ5 cm odom cellは同じworld bucketであり、同一物理点の再観測を保証しない",
            "並進/回転/ray分解は観測差の代数分解で、いずれかを真因と確定しない",
            "weighted fusionの全履歴寄与は保存されておらず、before/afterは各update時の値",
            "平坦路面という仮定を置く比較は実測ground truthではない",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2), encoding="utf-8")
    total_pairs = sum(len(cell["consecutive_accepted_observation_deltas"])
                      for target in target_results for cell in target["support_cells"])
    print("black targets:", len(target_results), "support cells:",
          sum(len(target["support_cells"]) for target in target_results),
          "accepted update pairs:", total_pairs,
          "missing candidate pixels:", len(missing_candidates))
    print("saved:", args.output)


if __name__ == "__main__":
    main()
