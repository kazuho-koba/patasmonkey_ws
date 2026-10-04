#!/usr/bin/env python3
"""前方半円内で独立frameの地形指標を評価し、空間地図へ期限なしで保持する。

秒ベースの評価scriptとは別ファイル。高さ融合・補間・通常nodeの変更は行わない。
保存済みmapper TFを必須にし、撮像stampを保持するが時間による破棄はしない。
"""
import argparse
import csv
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from evaluate_frame_feature_coverage import CUES, describe as describe_basic, aggregate, footprint_cells
from evaluate_independent_depth_frames import recorded_transform, pose_for_footprint
from evaluate_single_depth_terrain import messages, stamp_ns, evaluate_points
from pm_perception.depth_projection import sampled_points, transform_points
from pm_perception.mapper_replay_trace import load_mapper_trace


def describe(values, limits):
    """既存の黒判定と同じ100への丸めで、cue別の黒セル数も保存する。

    footprint内の別セルが別cueで黒の場合も各cueの関与として数える。
    原因の重複を保持し、最大寄与cueへの排他的割当は行わない。
    """
    result = describe_basic(values, limits)
    black = np.isfinite(values) & (np.rint(np.clip(values/limits, 0, 1)*100) >= 100)
    result.update({name+"_black_cells": int(black[:, i].sum()) for i, name in enumerate(CUES)})
    return result


def cause_summary(rows):
    """通過footprintの原因別関与数と排他的な原因組合せを集計する。"""
    counts = {name: sum(r[name+"_black_cells"] > 0 for r in rows) for name in CUES}
    combinations = {}
    for row in rows:
        names = [name for name in CUES if row[name+"_black_cells"] > 0]
        if names:
            key = "+".join(names)
            combinations[key] = combinations.get(key, 0)+1
    return dict(footprints=len(rows), black_footprints=sum(r["black_cells"] > 0 for r in rows),
        overlapping_cue_footprints=counts, exclusive_combinations=combinations,
        repeated_black_cells_by_cue={name: sum(r[name+"_black_cells"] for r in rows) for name in CUES})


def front_cells(pose, resolution, radius):
    """自車XYからradius[m]以内かつyaw前方の、odom格子cell中心を全列挙する。"""
    _, x, y, _, _, _, yaw = pose
    yy, xx = np.mgrid[int(np.floor((y-radius)/resolution)):int(np.ceil((y+radius)/resolution)),
                     int(np.floor((x-radius)/resolution)):int(np.ceil((x+radius)/resolution))]
    dx, dy = (xx+.5)*resolution-x, (yy+.5)*resolution-y
    valid = (dx*dx+dy*dy <= radius*radius) & (dx*np.cos(yaw)+dy*np.sin(yaw) >= 0)
    return list(zip(xx[valid].tolist(), yy[valid].tolist()))


class SpatialFeatureStore:
    """観測済みcellを無期限保持。将来の鮮度・pose整合性はeligibleで拡張できる。"""

    def __init__(self):
        self.data = {}

    def update(self, cells, values, stamp, support, coherent=True):
        """有効cueだけ更新し、unknownを過去の値の消去に使わない。"""
        for cell, row, quality in zip(cells, values, support):
            valid = np.isfinite(row)
            if coherent and not valid[:3].all():
                valid[:3] = False
            if not valid.any():
                continue
            if cell not in self.data:
                self.data[cell] = (np.full(4, np.nan), np.full(4, -1, dtype=np.int64),
                                   np.full(4, np.nan))
            kept, stamps, supports = self.data[cell]
            kept[valid], stamps[valid], supports[valid] = row[valid], stamp, quality[valid]

    def eligible(self, values, stamps, now, pose):
        """現在はfuture禁止のみ。時間経過・距離・通過済みを理由に失効しない。

        将来の動的物体/freshness/pose訂正ではこの判定を拡張できる。履歴stampは
        保存traceのframe poseと結び付く。現在の値が正しい・安全という保証ではない。
        """
        return np.isfinite(values) & (stamps >= 0) & (stamps <= now)

    def query(self, cells, now, pose, reference_max_age=None):
        values = np.full((len(cells), 4), np.nan)
        for i, cell in enumerate(cells):
            if cell in self.data:
                row, stamps, _ = self.data[cell]
                valid = self.eligible(row, stamps, now, pose)
                # 同一入力・締切で期限撤廃の効果だけを見る診断用参照条件。
                # 実際の保持storeは削除せず、既定Noneの無期限方式へ影響を与えない。
                if reference_max_age is not None:
                    valid &= now-stamps <= round(reference_max_age*1e9)
                values[i, valid] = row[valid]
        return values


def extract(arrays, cells, obstacle_clear_height=None, obstacle_clear_min_pixels=2):
    """独立画像mapから指定cellを抽出。map外もunknownとして分母に残す。"""
    result = np.full((len(cells), 4), np.nan)
    support = result.copy()
    origin = np.rint(arrays["origin"]/float(arrays["resolution"])).astype(int)
    for i, (x, y) in enumerate(cells):
        col, row = x-origin[0], y-origin[1]
        if 0 <= row < arrays["hazard"].shape[0] and 0 <= col < arrays["hazard"].shape[1]:
            result[i] = [arrays[name][row, col] for name in CUES]
            support[i] = [arrays["support_count"][row, col]]*2 + [
                min(arrays["step_support_forward"][row, col], arrays["step_support_rear"][row, col]),
                arrays["pixel_count"][row, col]]
            if obstacle_clear_height is not None:
                # unknown自体では解除しない。複数の実測点で低い高さ幅を確認した
                # セルだけ0を最新の証拠として保持する。安全なfree-spaceの保証ではない。
                span = arrays["cell_max"][row, col]-arrays["cell_min"][row, col]
                if (support[i, 3] >= obstacle_clear_min_pixels and np.isfinite(span)
                        and 0 <= span < obstacle_clear_height):
                    result[i, 3] = 0.0
    return result, support


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--mapper-trace", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--radius", type=float, default=2.0)
    parser.add_argument("--projection-map-size", type=float,
                        help="offlineだけの正方形投影領域[m]。半径比較では両条件を同じサイズにする")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--path-spacing", type=float, default=.25)
    parser.add_argument("--width", type=float, default=.45)
    parser.add_argument("--length", type=float, default=.55)
    parser.add_argument("--single-frame-obstacle", action="store_true")
    parser.add_argument("--legacy-obstacle-retention", action="store_true",
                        help="旧比較用：低い高さ幅で再観測しても過去obstacleを解除しない")
    parser.add_argument("--obstacle-clear-min-pixels", type=int, default=2,
                        help="低い高さ幅でobstacleを0に更新するための最低画素数（既定2）")
    parser.add_argument("--cue-wise", action="store_true", help="地形3cueの同一frame組保持ではなくcue別更新")
    parser.add_argument("--compare-age-seconds", type=float,
                        help="無期限方式は維持し、同一履歴に時間期限を付けた診断参照を追加")
    args = parser.parse_args()
    if min(args.radius, args.path_spacing, args.width, args.length) <= 0 or args.stride < 1:
        parser.error("距離・寸法・strideは正の値")
    if args.compare_age_seconds is not None and args.compare_age_seconds <= 0:
        parser.error("参照期限は正の秒数")
    if args.obstacle_clear_min_pixels < 2:
        parser.error("解除の最低画素数は2以上")
    output = Path(args.output); output.mkdir(parents=True, exist_ok=False)
    meta, fusions, _ = load_mapper_trace(args.mapper_trace)
    params = meta["parameters"]
    if args.projection_map_size is not None:
        if not np.isfinite(args.projection_map_size) or args.projection_map_size <= 0:
            parser.error("投影領域は正の有限値")
        # TFやcell幅は変えず、ROI外周のsupportまで保持できる一時gridの広さだけ変更。
        params = dict(params, map_size_x=args.projection_map_size, map_size_y=args.projection_map_size)
    if params.get("feature_max_accepted_ground_age", 0) > 0:
        raise ValueError("ground受理時刻gateは未対応")
    resolution = params["resolution"]
    limits = np.array([params["hazard_slope_limit_deg"], params["hazard_roughness_limit"],
                       params["hazard_step_limit"], params["hazard_obstacle_height_limit"]])
    path = []
    for stamp, frame in fusions.items():
        base = frame["base_to_map"]
        pose = pose_for_footprint(stamp, base["translation"]+base["quaternion"])
        if not path or np.hypot(pose[1]-path[-1][1], pose[2]-path[-1][2]) >= args.path_spacing:
            path.append(pose)
    footprints = [footprint_cells(p, resolution, args.width, args.length) for p in path]
    store = SpatialFeatureStore()
    entered, passed, seen, frame_rows, path_rows, age_rows = set(), set(), [], [], [], []
    start = time.perf_counter()
    for _, _, record in messages(args.bag, [params["depth_topic"]]):
        depth = deserialize_message(record.data, Image); stamp = stamp_ns(depth)
        if stamp not in fusions:
            continue
        frame = fusions[stamp]
        if frame["map_frame"] != "odom" or frame["image_frame"] != depth.header.frame_id or (depth.width, depth.height) != (frame["width"], frame["height"]):
            raise ValueError("traceとbagのframe/寸法不一致")
        seen.append(stamp)
        base = frame["base_to_map"]
        pose = pose_for_footprint(stamp, base["translation"]+base["quaternion"])
        # 通過時と同stampの画像もfuture扱い。通過前に保持された情報だけ評価する。
        for i, target in enumerate(path):
            if i not in passed and target[0] <= stamp:
                passed.add(i)
                path_rows.append(dict(path_index=i, event="before_passage", passage_stamp_ns=int(target[0]),
                    evaluation_stamp_ns=stamp, **describe(store.query(footprints[i], int(target[0])-1, target), limits)))
                if args.compare_age_seconds is not None:
                    age_rows.append(dict(path_rows[-1], **describe(store.query(
                        footprints[i], int(target[0])-1, target, args.compare_age_seconds), limits)))
        optical = sampled_points(depth, *frame["intrinsics"], args.stride, params["min_depth"], params["max_depth"])
        points = transform_points(optical, recorded_transform(frame["camera_to_map"]))
        arrays = evaluate_points(points, params, stamp, center=base["translation"][:2], heading_yaw=pose[-1],
            relative_elevation_offset=params["nominal_camera_height_above_ground"]-frame["camera_to_map"]["translation"][2],
            observation_variance=params["measurement_variance"]+params["depth_variance_per_meter_sq"]*np.sum(optical*optical, axis=1))
        if args.single_frame_obstacle:
            arrays["obstacle_height"] = arrays["raw_obstacle"]
        roi = front_cells(pose, resolution, args.radius)
        current, support = extract(arrays, roi,
            obstacle_clear_height=None if args.legacy_obstacle_retention else params["obstacle_min_height"],
            obstacle_clear_min_pixels=args.obstacle_clear_min_pixels)
        store.update(roi, current, stamp, support, coherent=not args.cue_wise)
        for mode, values in (("independent", current), ("held", store.query(roi, stamp, pose))):
            frame_rows.append(dict(frame_stamp_ns=stamp, mode=mode, **describe(values, limits)))
        for i, target in enumerate(path):
            dx, dy = target[1]-pose[1], target[2]-pose[2]
            if (i not in entered and target[0] > stamp and dx*dx+dy*dy <= args.radius**2
                    and dx*np.cos(pose[-1])+dy*np.sin(pose[-1]) >= 0):
                entered.add(i)
                path_rows.append(dict(path_index=i, event="first_front_roi_entry", passage_stamp_ns=int(target[0]),
                    evaluation_stamp_ns=stamp, distance_to_passage_m=float(np.hypot(dx, dy)),
                    **describe(store.query(footprints[i], stamp, pose), limits)))
                if args.compare_age_seconds is not None:
                    age_rows.append(dict(path_rows[-1], **describe(store.query(
                        footprints[i], stamp, pose, args.compare_age_seconds), limits)))
        if len(seen) % 200 == 0:
            print("evaluated %d frames" % len(seen), flush=True)
    if seen != list(fusions):
        raise ValueError("traceとbagの採用stamp/順序不一致")
    summary = dict(parameters=params, radius_m=args.radius, stride=args.stride,
        retention="unlimited spatial store; no time/distance eviction", coherent_terrain=not args.cue_wise,
        single_frame_obstacle=args.single_frame_obstacle, adopted_frames=len(seen), path_samples=len(path),
        obstacle_update=dict(clear_low_span=not args.legacy_obstacle_retention,
            min_pixels=args.obstacle_clear_min_pixels, clear_height_below_m=params["obstacle_min_height"],
            not_a_free_space_guarantee=True),
        footprint_width=args.width, footprint_length=args.length, path_spacing=args.path_spacing,
        retained_cells=len(store.data), elapsed_seconds=time.perf_counter()-start,
        front_roi={mode: aggregate([r for r in frame_rows if r["mode"] == mode]) for mode in ("independent", "held")},
        path={event: aggregate([r for r in path_rows if r["event"] == event]) for event in ("first_front_roi_entry", "before_passage")},
        path_causes={event: cause_summary([r for r in path_rows if r["event"] == event])
                     for event in ("first_front_roi_entry", "before_passage")},
        compare_age_seconds=args.compare_age_seconds,
        path_age_reference={event: aggregate([r for r in age_rows if r["event"] == event])
                            for event in ("first_front_roi_entry", "before_passage")},
        trace_sha256=hashlib.sha256(Path(args.mapper_trace).read_bytes()).hexdigest(),
        mapper_trace=str(Path(args.mapper_trace).resolve()), bag=str(Path(args.bag).resolve()))
    (output/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
    for filename, rows in (("front_roi.csv", frame_rows), ("path_footprints.csv", path_rows),
                           ("age_reference_footprints.csv", age_rows)):
        with (output/filename).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
            writer.writeheader(); writer.writerows(rows)
    cells = sorted(store.data)
    np.savez_compressed(output/"retained_features.npz", cells=np.asarray(cells, dtype=np.int64),
        values=np.asarray([store.data[c][0] for c in cells]),
        stamp_ns=np.asarray([store.data[c][1] for c in cells], dtype=np.int64),
        support=np.asarray([store.data[c][2] for c in cells]), resolution=resolution)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
