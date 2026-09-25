#!/usr/bin/env python3
"""黒hazardの3x3 support内で、mapperのmin-z ground候補が孤立しているか調べる。

各black targetのplane_support.csvが参照するsupport cellとlatest source stampを使い、
そのframe/cellへ入ったframe_pixelsの全stride sampleを集める。mapperが採用したmin-z
候補をcell内z分布の中央値・分位点・近傍sample数と比較し、単独の低いtail sampleが
局所planeの高さを支配していないかをオフラインで確認する。

これは同一cell内のsample分布検査で、深度誤差のground truth判定ではない。セル内の
world-z幅には、実路面傾斜、同じ5 cm bin内の実XY分布、pose/ray誤差、depthノイズが
全て含まれ得る。分位点閾値は結果表示用であり、mapperの採用条件を再現/変更しない。
"""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


def read_csv(path):
    """CSVを読み込む。対象は前方ROIの診断データに限定されている。"""
    with Path(path).open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def summarize_samples(samples, cell, expected_count):
    """同一frame/cellのworld-z候補分布とmin候補の位置を要約する。"""
    samples = sorted(samples, key=lambda row: float(row["world_z_m"]))
    z = np.asarray([float(row["world_z_m"]) for row in samples], dtype=np.float64)
    minimum = float(z[0])
    median = float(np.median(z))
    min_row = samples[0]
    points_xy = np.asarray([[float(row["world_x_m"]), float(row["world_y_m"])]
                            for row in samples], dtype=np.float64)
    cell_center = np.asarray([(cell[0] + 0.5) * 0.05,
                              (cell[1] + 0.5) * 0.05], dtype=np.float64)
    offset = points_xy - cell_center
    return {
        "sample_count_in_pixels_csv": len(samples),
        "sample_count_from_frame_event": expected_count,
        "count_matches_frame_event": len(samples) == expected_count,
        "world_z_min_m": minimum,
        "world_z_p10_m": float(np.percentile(z, 10)),
        "world_z_p25_m": float(np.percentile(z, 25)),
        "world_z_median_m": median,
        "world_z_p75_m": float(np.percentile(z, 75)),
        "world_z_p90_m": float(np.percentile(z, 90)),
        "world_z_max_m": float(z[-1]),
        "min_to_median_gap_m": median - minimum,
        "min_to_p10_gap_m": float(np.percentile(z, 10)) - minimum,
        "samples_within_min_plus_0p01m": int(np.count_nonzero(z <= minimum + 0.01)),
        "samples_within_min_plus_0p02m": int(np.count_nonzero(z <= minimum + 0.02)),
        "samples_within_min_plus_0p05m": int(np.count_nonzero(z <= minimum + 0.05)),
        "min_pixel_uv": [int(min_row["pixel_u"]), int(min_row["pixel_v"])],
        "min_pixel_depth_m": float(min_row["axial_depth_m"]),
        "min_point_world_xy_m": points_xy[0].tolist(),
        "cell_center_offset_of_min_xy_m": offset[0].tolist(),
        "cell_sample_xy_extent_m": [
            float(np.max(points_xy[:, axis]) - np.min(points_xy[:, axis]))
            for axis in range(2)],
        "all_sample_pixels_uv": [[int(row["pixel_u"]), int(row["pixel_v"])]
                                  for row in samples],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("forensic_dir", type=Path)
    parser.add_argument("color_projection_summary", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    targets = {int(row["index"]): row for row in json.loads(
        args.color_projection_summary.read_text(encoding="utf-8"))["targets"]
        if int(row["index"]) in (5, 12, 13)}
    supports = read_csv(args.forensic_dir / "plane_support.csv")
    events = read_csv(args.forensic_dir / "frame_events.csv")
    pixels = read_csv(args.forensic_dir / "frame_pixels.csv")

    # target indexごとにblack中心のmap stamp・cellへ一致する3x3 supportを取る。
    support_rows = {}
    for index, target in targets.items():
        key = (int(target["map_stamp_ns"]), int(target["odom_cell_x"]),
               int(target["odom_cell_y"]))
        support_rows[index] = [row for row in supports
                               if (int(row["map_stamp_ns"]),
                                   int(row["target_cell_x"]),
                                   int(row["target_cell_y"])) == key]
        if len(support_rows[index]) != 9:
            raise RuntimeError("target {}のplane support数が9でありません".format(index))

    # 最新source frameごとのframe eventを用いてmin候補のsupport数を照合する。
    event_by_key = {(int(row["source_stamp_ns"]), int(row["odom_cell_x"]),
                     int(row["odom_cell_y"])): row for row in events}
    desired = set()
    for rows in support_rows.values():
        desired.update((int(row["latest_source_stamp_ns"]),
                        int(row["support_cell_x"]), int(row["support_cell_y"]))
                       for row in rows)
    samples_by_key = defaultdict(list)
    for row in pixels:
        key = (int(row["source_stamp_ns"]), int(row["odom_cell_x"]),
               int(row["odom_cell_y"]))
        if key in desired:
            samples_by_key[key].append(row)

    output_targets = []
    for index, target in targets.items():
        cell_results = []
        for support in sorted(support_rows[index],
                              key=lambda row: (int(row["support_cell_y"]),
                                               int(row["support_cell_x"]))):
            stamp = int(support["latest_source_stamp_ns"])
            cell = (int(support["support_cell_x"]), int(support["support_cell_y"]))
            key = (stamp, cell[0], cell[1])
            event = event_by_key.get(key)
            samples = samples_by_key.get(key, [])
            cell_results.append({
                "cell": list(cell), "latest_source_stamp_ns": stamp,
                "source_age_s": float(support["age_s"]),
                "map_relative_elevation_m": float(support["relative_elevation_m"]),
                "plane_residual_m": float(support["plane_residual_m"]),
                "fusion_mode": (int(event["fusion_mode"]) if event else None),
                "fusion_ground_before_m": (float(event["ground_before_m"])
                                            if event else None),
                "fusion_ground_after_m": (float(event["ground_after_m"])
                                           if event else None),
                "candidate_distribution": summarize_samples(
                    samples, cell, int(event["sample_count"]) if event else None)
                    if samples else None,
            })
        output_targets.append({
            "target_index": index, "map_stamp_ns": int(target["map_stamp_ns"]),
            "hazard_cell": [int(target["odom_cell_x"]), int(target["odom_cell_y"])],
            "max_cause": int(target["max_cause"]),
            "slope_deg": float(target["slope_deg"]),
            "step_m": (float(target["step_m"]) if target.get("step_m") else None),
            "support_height_range_m": float(target["support_map_height_range_m"]),
            "support_cells": cell_results,
        })

    all_cells = [cell for target in output_targets for cell in target["support_cells"]]
    candidate_gaps = [cell["candidate_distribution"]["min_to_median_gap_m"]
                      for cell in all_cells if cell["candidate_distribution"]]
    output = {
        "target_count": len(output_targets),
        "support_cell_count": len(all_cells),
        "cells_with_candidate_distribution": len(candidate_gaps),
        "cells_with_pixel_count_mismatch": sum(
            not cell["candidate_distribution"]["count_matches_frame_event"]
            for cell in all_cells if cell["candidate_distribution"]),
        "min_to_median_gap_summary_m": ({
            "median": float(np.median(candidate_gaps)),
            "p90": float(np.percentile(candidate_gaps, 90)),
            "max": float(np.max(candidate_gaps)),
            "cells_over_0p02m": int(np.count_nonzero(np.asarray(candidate_gaps) > 0.02)),
        } if candidate_gaps else None),
        "targets": output_targets,
        "limitations": [
            "min候補とcell内medianの差はdepth外れ値の確定判定ではない",
            "cell内world-z幅は地面勾配、サブセル位置、姿勢/ray誤差、depth誤差を含み得る",
            "候補sampleはpixel_strideで間引かれた点だけである",
            "この診断はground融合のmin-z候補と plane supportの最新frameを評価し、過去の重み全履歴は復元しない",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2), encoding="utf-8")
    print("targets:", len(output_targets), "support cells:", len(all_cells),
          "with samples:", len(candidate_gaps),
          "count mismatches:", output["cells_with_pixel_count_mismatch"])
    print("min-to-median gap:", output["min_to_median_gap_summary_m"])
    print("saved:", args.output)


if __name__ == "__main__":
    main()
