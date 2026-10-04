#!/usr/bin/env python3
"""実際のmapper TFで独立画像の指標を保持する、A相当のoffline coverage試験。

高さ融合・空間補間・z補正は行わない。通過時刻よりlead秒前を評価締切にし、
その締切以前の画像だけで3方式を同時評価する。未来のposeは経路ラベルにのみ使う。
通常perceptionの挙動やROS通信は変更しない。
"""
import argparse
import csv
import hashlib
import json
import resource
import time
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

from evaluate_independent_depth_frames import recorded_transform, pose_for_footprint
from evaluate_single_depth_terrain import messages, stamp_ns, evaluate_points
from pm_perception.depth_projection import sampled_points, transform_points
from pm_perception.mapper_replay_trace import load_mapper_trace
from pm_perception.rolling_elevation_grid import RollingElevationGrid


CUES = ("slope_deg", "roughness", "step_height", "obstacle_height")


def footprint_cells(pose, resolution, width, length):
    """map外も分母から除かず、odom格子のcell中心で長方形footprintを列挙する。"""
    _, x, y, _, _, _, yaw = pose
    radius = np.hypot(width, length)/2
    yy, xx = np.mgrid[int(np.floor((y-radius)/resolution)):int(np.ceil((y+radius)/resolution)),
                     int(np.floor((x-radius)/resolution)):int(np.ceil((x+radius)/resolution))]
    dx, dy = (xx+.5)*resolution-x, (yy+.5)*resolution-y
    inside = ((np.abs(dx*np.cos(yaw)+dy*np.sin(yaw)) <= length/2) &
              (np.abs(-dx*np.sin(yaw)+dy*np.cos(yaw)) <= width/2))
    return list(zip(xx[inside].tolist(), yy[inside].tolist()))


class FeatureMemory:
    """固定した経路周辺セルだけを保持し、全画像mapの保存を避ける。"""

    def __init__(self, cells, limits):
        self.cells = cells
        self.lookup = {cell: i for i, cell in enumerate(cells)}
        self.limits = np.asarray(limits)
        self.values = np.full((len(cells), 4), np.nan)
        self.stamps = np.full((len(cells), 4), -1, dtype=np.int64)
        self.first = np.full((len(cells), 4), -1, dtype=np.int64)
        # endpointが同じ画像でplane/step support付き評価できた最後の時刻。
        # 実際の段差測定そのものではなく「継ぎ目support」の保守的なproxyである。
        self.edges = {}

    def extract(self, arrays):
        """論理map座標を絶対odom-cellへ結び、領域外はunknownのまま返す。"""
        xy = np.asarray(self.cells)
        origin = np.rint(arrays["origin"]/float(arrays["resolution"])).astype(int)
        ix, iy = xy[:, 0]-origin[0], xy[:, 1]-origin[1]
        h, w = arrays["hazard"].shape
        inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        values = np.full_like(self.values, np.nan)
        for c, name in enumerate(CUES):
            values[inside, c] = arrays[name][iy[inside], ix[inside]]
        return values

    def update(self, values, stamp):
        """有効なcueだけ最新値に更新。unknownは過去の有効値を消さない。"""
        valid = np.isfinite(values)
        initial = valid & (self.first < 0)
        self.first[initial] = stamp
        self.values[valid], self.stamps[valid] = values[valid], stamp
        terrain = np.all(valid[:, :3], axis=1)
        for i in np.flatnonzero(terrain):
            x, y = self.cells[i]
            for neighbor in ((x+1, y), (x, y+1)):
                j = self.lookup.get(neighbor)
                if j is not None and terrain[j]:
                    self.edges[(i, j)] = stamp

    def query(self, cutoff, age):
        """cueごとに保持期限を適用。異なるframe由来の高さは参照しない。"""
        fresh = ((self.stamps >= 0) & (self.stamps <= cutoff) &
                 (cutoff-self.stamps <= round(age*1e9)))
        return np.where(fresh, self.values, np.nan)


def describe(values, limits):
    """hazard既知とcue完全評価を分離。100への表示丸めを従来と揃える。"""
    valid = np.isfinite(values)
    known = valid.any(axis=1)
    black = (valid & (np.rint(np.clip(values/limits, 0, 1)*100) >= 100)).any(axis=1)
    return dict(cells=len(values), known_cells=int(known.sum()), black_cells=int(black.sum()),
                terrain_cells=int(valid[:, :3].all(axis=1).sum()),
                all_cue_cells=int(valid.all(axis=1).sum()),
                **{name+"_valid_cells": int(valid[:, i].sum()) for i, name in enumerate(CUES)})


def aggregate(rows):
    """footprint黒率の分母とunknownのセル率・footprint率を明示する。"""
    total = sum(r["cells"] for r in rows)
    known = sum(r["known_cells"] for r in rows)
    known_footprints = sum(r["known_cells"] > 0 for r in rows)
    n = len(rows)
    return dict(footprints=n, cells=total,
                known_footprints=known_footprints,
                completely_unknown=sum(r["known_cells"] == 0 for r in rows),
                partial_unknown=sum(0 < r["known_cells"] < r["cells"] for r in rows),
                fully_known=sum(r["known_cells"] == r["cells"] for r in rows),
                black_footprints=sum(r["black_cells"] > 0 for r in rows),
                black_percent_of_known=(100*sum(r["black_cells"] > 0 for r in rows)/known_footprints
                                        if known_footprints else None),
                unknown_cell_percent=100*(total-known)/total if total else None,
                fully_terrain_footprints=sum(r["terrain_cells"] == r["cells"] for r in rows),
                fully_all_cue_footprints=sum(r["all_cue_cells"] == r["cells"] for r in rows),
                **{key: 100*sum(r[key] for r in rows)/total if total else None
                   for key in ("terrain_cells", "all_cue_cells") +
                   tuple(name+"_valid_cells" for name in CUES)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--mapper-trace", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--lead-seconds", type=float, nargs="+", default=[0.2, 1.0, 2.0])
    parser.add_argument("--max-age", type=float, default=3.0)
    parser.add_argument("--path-spacing", type=float, default=0.25)
    parser.add_argument("--width", type=float, default=0.45)
    parser.add_argument("--length", type=float, default=0.55)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--single-frame-obstacle", action="store_true")
    args = parser.parse_args()
    if min(args.lead_seconds+[args.max_age, args.path_spacing, args.width, args.length]) <= 0 or args.stride < 1:
        parser.error("時間・寸法・strideは正の値")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    metadata, fusions, snapshots = load_mapper_trace(args.mapper_trace)
    params = dict(metadata["parameters"])
    if params.get("feature_max_accepted_ground_age", 0) > 0:
        raise ValueError("ground受理時刻gateは未対応")
    resolution = params["resolution"]
    limits = np.array([params["hazard_slope_limit_deg"], params["hazard_roughness_limit"],
                       params["hazard_step_limit"], params["hazard_obstacle_height_limit"]])
    # 経路も同じtraceのbase poseから作る。別再生のodomを混ぜない。
    path = []
    for stamp, frame in fusions.items():
        p = frame["base_to_map"]
        pose = pose_for_footprint(stamp, p["translation"]+p["quaternion"])
        if not path or np.hypot(pose[1]-path[-1][1], pose[2]-path[-1][2]) >= args.path_spacing:
            path.append(pose)
    footprints = [footprint_cells(pose, resolution, args.width, args.length) for pose in path]
    cells = sorted(set(cell for group in footprints for cell in group))
    memory = FeatureMemory(cells, limits)
    coherent = FeatureMemory(cells, limits)
    requests = sorted((int(pose[0])-round(lead*1e9), i, lead)
                      for i, pose in enumerate(path) for lead in args.lead_seconds)
    cursor, seen, rows, timings = 0, [], [], []
    current = np.full_like(memory.values, np.nan)
    temporal = current.copy()
    last_stamp = -1
    grid = RollingElevationGrid(params["map_size_x"], params["map_size_y"], resolution)

    def emit(request):
        """締切時点の状態だけ抽出する。未来frameのupdate前に呼ぶ。"""
        cutoff, index, lead = request
        ids = np.array([memory.lookup[cell] for cell in footprints[index]], dtype=int)
        held = memory.query(cutoff, args.max_age)
        packet = coherent.query(cutoff, args.max_age)
        latest = current if last_stamp >= 0 and cutoff-last_stamp <= args.max_age*1e9 else np.full_like(current, np.nan)
        fused = temporal if last_stamp >= 0 and cutoff-last_stamp <= args.max_age*1e9 else np.full_like(temporal, np.nan)
        for mode, values in (("independent", latest), ("held_features", held),
                             ("held_coherent_terrain", packet), ("temporal", fused)):
            row = dict(path_index=index, passage_stamp_ns=int(path[index][0]),
                       cutoff_stamp_ns=cutoff, lead_seconds=lead, mode=mode,
                       latest_frame_stamp_ns=last_stamp, **describe(values[ids], limits))
            if mode == "held_features":
                terrain = np.isfinite(held[:, :3]).all(axis=1)
                terrain_ids = ids[terrain[ids]]
                ages = (cutoff-memory.stamps[terrain_ids, :3])/1e9
                row["oldest_terrain_age_seconds"] = float(ages.max()) if ages.size else None
                # 各cueが異なる画像で有効になった場合も区別する。
                row["coherent_terrain_cells"] = int(sum(
                    len(set(memory.stamps[i, :3])) == 1 for i in terrain_ids))
                pairs, unsupported, seams = 0, 0, 0
                id_set = set(ids)
                for i in ids:
                    x, y = cells[i]
                    for cell in ((x+1, y), (x, y+1)):
                        j = memory.lookup.get(cell)
                        if j in id_set and terrain[i] and terrain[j]:
                            pairs += 1
                            seams += int(not np.array_equal(memory.stamps[i, :3], memory.stamps[j, :3]))
                            stamp = memory.edges.get((i, j), -1)
                            unsupported += int(stamp < 0 or cutoff-stamp > args.max_age*1e9)
                row.update(terrain_neighbor_pairs=pairs, different_source_pairs=seams,
                           no_fresh_joint_support_pairs=unsupported)
                ever = (memory.stamps[ids] >= 0).any(axis=1)
                known = np.isfinite(held[ids]).any(axis=1)
                row.update(never_evaluated_cells=int((~ever).sum()),
                           expired_only_cells=int((ever & ~known).sum()))
            rows.append(row)

    started = time.perf_counter()
    cpu_started = time.process_time()
    for _, _, record in messages(args.bag, [params["depth_topic"]]):
        depth = deserialize_message(record.data, Image)
        stamp = stamp_ns(depth)
        if stamp not in fusions:
            continue
        # 評価締切と同stampの画像は利用可。それより未来の画像は決して使わない。
        while cursor < len(requests) and requests[cursor][0] < stamp:
            emit(requests[cursor]); cursor += 1
        frame = fusions[stamp]
        if frame["map_frame"] != "odom" or frame["image_frame"] != depth.header.frame_id or (frame["width"], frame["height"]) != (depth.width, depth.height):
            raise ValueError("bagとtraceのframe/寸法不一致")
        seen.append(stamp)
        tick = time.perf_counter()
        optical = sampled_points(depth, *frame["intrinsics"], args.stride, params["min_depth"], params["max_depth"])
        points = transform_points(optical, recorded_transform(frame["camera_to_map"]))
        pose = frame["base_to_map"]
        yaw = pose_for_footprint(stamp, pose["translation"]+pose["quaternion"])[-1]
        kwargs = dict(center=pose["translation"][:2], heading_yaw=yaw,
                      relative_elevation_offset=params["nominal_camera_height_above_ground"]-frame["camera_to_map"]["translation"][2],
                      observation_variance=params["measurement_variance"]+params["depth_variance_per_meter_sq"]*np.sum(optical*optical, axis=1))
        independent = evaluate_points(points, params, stamp, **kwargs)
        if args.single_frame_obstacle:
            independent["obstacle_height"] = independent["raw_obstacle"]
        current = memory.extract(independent)
        memory.update(current, stamp)
        # 3cueを同じframeで評価できた組だけ更新する対照。obstacleは独立channel。
        packet_values = current.copy()
        complete = np.isfinite(current[:, :3]).all(axis=1)
        packet_values[~complete, :3] = np.nan
        coherent.update(packet_values, stamp)
        temporal = memory.extract(evaluate_points(points, params, stamp, grid=grid, **kwargs))
        last_stamp = stamp
        timings.append((time.perf_counter()-tick)*1000)
        if len(seen) % 200 == 0:
            print("evaluated %d frames" % len(seen), flush=True)
    if seen != list(fusions):
        raise ValueError("bagの画像stamp/順序とtraceが一致しません")
    while cursor < len(requests):
        emit(requests[cursor]); cursor += 1
    results = {}
    for lead in args.lead_seconds:
        results[str(lead)] = {mode: aggregate([r for r in rows if r["lead_seconds"] == lead and r["mode"] == mode])
                             for mode in ("independent", "held_features", "held_coherent_terrain", "temporal")}
    held_rows = [r for r in rows if r["mode"] == "held_features"]
    leads = []
    for pose, group in zip(path, footprints):
        for cell in group:
            i = memory.lookup[cell]
            stamps = memory.first[i, :3]
            if np.all(stamps >= 0) and np.max(stamps) < pose[0]:
                leads.append((int(pose[0])-int(np.max(stamps)))/1e9)
    summary = dict(parameters=params, stride=args.stride, max_age_seconds=args.max_age,
                   single_frame_obstacle=args.single_frame_obstacle, adopted_frames=len(seen),
                   path_samples=len(path), target_cells=len(cells), results=results,
                   joint_support=dict(terrain_neighbor_pairs=sum(r["terrain_neighbor_pairs"] for r in held_rows),
                                      different_source_pairs=sum(r["different_source_pairs"] for r in held_rows),
                                      no_fresh_joint_support_pairs=sum(r["no_fresh_joint_support_pairs"] for r in held_rows)),
                   first_terrain_cue_available_lead_seconds=dict(count=len(leads),
                        median=float(np.median(leads)) if leads else None,
                        p05=float(np.percentile(leads, 5)) if leads else None),
                   elapsed_seconds=time.perf_counter()-started,
                   process_cpu_seconds=time.process_time()-cpu_started, peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                   frame_ms=dict(mean=float(np.mean(timings)), p95=float(np.percentile(timings, 95)), max=float(max(timings))),
                   trace_sha256=hashlib.sha256(Path(args.mapper_trace).read_bytes()).hexdigest(),
                   bag=str(Path(args.bag).resolve()), mapper_trace=str(Path(args.mapper_trace).resolve()),
                   evaluation_schedule="each adopted image; latest state at fixed pre-passage cutoffs",
                   limitations=["coverage prototype: latest valid cue overwrite, not safety evidence fusion",
                                "obstacle unknown is not evidence of free space",
                                "edge support proxy does not prove physical boundary is safe",
                                "first availability ignores expiry; latest ages are separately recorded"])
    (output/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
    with (output/"footprints.csv").open("w", newline="") as stream:
        fields = list(dict.fromkeys(key for row in rows for key in row))
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
