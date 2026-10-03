#!/usr/bin/env python3
"""全depth frameを独立評価し、最新odom実績経路への黒重なり率を求める。

毎frame空gridを作る。ROS通信・時間融合はなく、stride=1が既定。
撮像時刻のposeは再計算済みCSVから位置線形補間・quaternion SLERPする。
フレーム頻度で重み付けされた率と、距離0.25 mごとの率を分離して出力する。
"""
import argparse
import csv
import json
from pathlib import Path
from types import SimpleNamespace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
import yaml

from evaluate_single_depth_terrain import messages, stamp_ns, evaluate_points
from evaluate_traversed_terrain import footprint_values
from pm_perception.depth_projection import quaternion_matrix, sampled_points, transform_points
from pm_perception.rolling_elevation_grid import RollingElevationGrid
from pm_perception.mapper_replay_trace import load_mapper_trace


def quaternion(q):
    return SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])


def recorded_transform(record):
    """保存されたTFを再補間せず、そのまま既存の点変換関数へ渡す。"""
    t = record["translation"]
    return SimpleNamespace(translation=SimpleNamespace(x=t[0], y=t[1], z=t[2]),
                           rotation=quaternion(record["quaternion"]))


def interpolate_pose(times, poses, stamp, max_gap):
    """外挿・大きい欠落区間は棄却。qと-qの同一回転を考慮してSLERPする。"""
    index = np.searchsorted(times, stamp)
    if index < len(times) and times[index] == stamp:
        return poses[index]
    if index == 0 or index == len(times) or times[index]-times[index-1] > max_gap*1e9:
        return None
    alpha = (stamp-times[index-1])/(times[index]-times[index-1])
    lo, hi = poses[index-1], poses[index]
    q0 = lo[3:]/np.linalg.norm(lo[3:])
    q1 = hi[3:]/np.linalg.norm(hi[3:])
    dot = np.dot(q0, q1)
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > .9995:
        q = q0 + alpha*(q1-q0)
    else:
        angle = np.arccos(np.clip(dot, -1, 1))
        q = (np.sin((1-alpha)*angle)*q0+np.sin(alpha*angle)*q1)/np.sin(angle)
    q /= np.linalg.norm(q)
    return np.r_[lo[:3]+alpha*(hi[:3]-lo[:3]), q]


def pose_for_footprint(stamp, pose):
    qx, qy, qz, qw = pose[3:]
    yaw = np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    return (int(stamp), *pose[:3], 0.0, 0.0, yaw)


def evaluate_footprint(arrays, pose, width, length, params, cell_diagnostics=False):
    """unknownを安全へ置換しない。component件数は重複あり。"""
    result = {}
    for layer in ["hazard", "reference_hazard"]:
        values = footprint_values(arrays[layer], arrays["origin"], params["resolution"], pose, length, width)
        known = np.isfinite(values)
        result[layer+"_cells"] = len(values)
        result[layer+"_known"] = int(known.sum())
        # 従来のRViz値100と比較するため、丸め後の100を同じ基準で数える。
        result[layer+"_black"] = int((known & (np.rint(np.clip(values, 0, 1)*100) >= 100)).sum())
    for name, limit in [("slope_deg", params["hazard_slope_limit_deg"]),
                        ("roughness", params["hazard_roughness_limit"]),
                        ("step_height", params["hazard_step_limit"]),
                        ("obstacle_height", params["hazard_obstacle_height_limit"])]:
        values = footprint_values(arrays[name], arrays["origin"], params["resolution"], pose, length, width)
        result[name+"_black_any"] = int(np.any(np.isfinite(values) & (np.rint(np.clip(values/limit, 0, 1)*100) >= 100)))
    if cell_diagnostics:
        # offline専用。採用footprintの各セルを同じodom位置で照合できるよう保存する。
        # map全体や画像を増やさず、最良frame選択後の少数セル情報だけCSVへ残す。
        shape = arrays["hazard"].shape
        indices = footprint_values(np.arange(np.prod(shape)).reshape(shape), arrays["origin"],
                                   params["resolution"], pose, length, width).astype(int)
        iy, ix = np.divmod(indices, shape[1])
        data = {"x": (arrays["origin"][0]+(ix+.5)*params["resolution"]).tolist(),
                "y": (arrays["origin"][1]+(iy+.5)*params["resolution"]).tolist()}
        for name in ("hazard", "reference_hazard", "slope_deg", "roughness", "step_height",
                     "obstacle_height", "pixel_count", "support_count", "step_support_forward",
                     "step_support_rear", "cell_min", "cell_max"):
            values = arrays[name].ravel()[indices]
            # JSON標準で欠測をnullにし、NaNを数値の安全値へ置き換えない。
            data[name] = [float(v) if np.isfinite(v) else None for v in values]
        result["cell_diagnostics_json"] = json.dumps(data, allow_nan=False, separators=(",", ":"))
    return result


def summarize(rows):
    """完全unknown・部分既知・完全既知を分け、分母を明示する。"""
    summary = {"matched_samples": len(rows)}
    for layer in ["hazard", "reference_hazard"]:
        known = [r for r in rows if r[layer+"_known"] > 0]
        full = [r for r in known if r[layer+"_known"] == r[layer+"_cells"]]
        black = sum(r[layer+"_black"] > 0 for r in known)
        black_full = sum(r[layer+"_black"] > 0 for r in full)
        total = sum(r[layer+"_cells"] for r in rows)
        observed = sum(r[layer+"_known"] for r in rows)
        summary[layer] = dict(known_footprints=len(known), fully_known=len(full),
                             completely_unknown=len(rows)-len(known), partial=len(known)-len(full),
                             black_any=black, black_percent=100*black/len(known) if known else None,
                             black_any_fully_known=black_full,
                             fully_known_black_percent=100*black_full/len(full) if full else None,
                             unknown_cell_percent=100*(total-observed)/total if total else None)
    for cue in ["slope_deg", "roughness", "step_height", "obstacle_height"]:
        summary[cue+"_black_any"] = sum(r[cue+"_black_any"] for r in rows)
    return summary


def write_csv(path, rows):
    if rows:
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)


def save_example(output, arrays, depth, stamp):
    """代表frameだけを保存し、全画像・全mapの大量保存は避ける。"""
    folder = output/("frame_"+str(stamp))
    folder.mkdir(exist_ok=True)
    np.savez_compressed(folder/"map.npz", **arrays)
    raw = np.frombuffer(depth.data, dtype=">u2" if depth.is_bigendian else "<u2").reshape(depth.height, depth.step//2)[:, :depth.width]
    np.save(folder/"depth_mm.npy", raw)
    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    for ax, name in zip(axes.flat, ["elevation", "slope_deg", "roughness", "step_height", "hazard", "reference_hazard"]):
        cmap = plt.get_cmap("gray_r" if "hazard" in name else "viridis").copy()
        cmap.set_bad("orchid")
        image = ax.imshow(arrays[name], origin="lower", cmap=cmap,
                          vmin=0 if "hazard" in name else None, vmax=1 if "hazard" in name else None)
        ax.set_title(name); fig.colorbar(image, ax=ax)
    fig.suptitle("Independent frame %d; unknown=purple" % stamp)
    fig.tight_layout(); fig.savefig(folder/"map.png", dpi=120); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--poses-csv", required=True)
    parser.add_argument("--mapper-trace", help="通常mapperのmapper_trace.jsonl。指定時は投影TFを再計算しない")
    parser.add_argument("--config", required=True)
    parser.add_argument("--override")
    parser.add_argument("--output", required=True)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--resolution", type=float)
    parser.add_argument("--temporal-fusion", action="store_true",
                        help="対照用。空gridにせずruntimeと同じgridを時間融合する")
    parser.add_argument("--footprint-cell-diagnostics", action="store_true",
                        help="offline専用。経路footprintのcue値・support・XYをCSVへ追加保存する")
    parser.add_argument("--single-frame-obstacle", action="store_true",
                        help="独立frameのmax−minをconfidence待ちなしでobstacleにし、reference_hazardを主指標とする")
    parser.add_argument("--camera-frame", default="rgb_camera_optical_frame")
    parser.add_argument("--max-pose-gap", type=float, default=.25)
    parser.add_argument("--width", type=float, default=.45)
    parser.add_argument("--length", type=float, default=.55)
    args = parser.parse_args()
    if args.stride < 1 or args.max_pose_gap <= 0:
        parser.error("stride/gapは正の値")
    if args.single_frame_obstacle and args.temporal_fusion:
        parser.error("単一frame obstacleオプションは独立frame評価用。時間融合対照では指定しない")
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(Path(args.config).read_text())
    # ros2 param dumpの絶対ノード名にも対応し、採取時の実効設定をそのまま読める。
    params = (config.get("depth_elevation_mapper") or config["/depth_elevation_mapper"])["ros__parameters"]
    if args.override:
        params.update(yaml.safe_load(Path(args.override).read_text())["depth_elevation_mapper"]["ros__parameters"])
    if args.resolution is not None:
        params["resolution"] = args.resolution
    trace_metadata, trace_fusions, trace_snapshots = (load_mapper_trace(args.mapper_trace)
        if args.mapper_trace else (None, None, []))
    if trace_metadata is not None:
        # 投影・融合・featureの条件差を黙って持ち込まない。strideだけは独立比較の因子。
        keys = ["map_size_x", "map_size_y", "resolution", "min_depth", "max_depth",
                "nominal_camera_height_above_ground", "ground_merge_threshold", "obstacle_min_height",
                "measurement_variance", "depth_variance_per_meter_sq", "observation_decay_time",
                "obstacle_confidence_min", "feature_max_observation_age", "feature_neighborhood_radius_cells",
                "feature_min_neighbors", "step_min_side_neighbors", "hazard_slope_limit_deg",
                "hazard_roughness_limit", "hazard_step_limit", "hazard_obstacle_height_limit"]
        differences = [key for key in keys if params[key] != trace_metadata["parameters"][key]]
        if differences:
            raise ValueError("traceと検証設定の不一致: " + ", ".join(differences))
        if trace_metadata["parameters"].get("feature_max_accepted_ground_age", 0) > 0:
            raise ValueError("ground受理時刻gateは本ツール未対応。黙って省略しません")
    # 同stamp重複は最後のpublishを残す。時刻をfloatに丸めずns整数として読む。
    unique = {}
    with open(args.poses_csv) as stream:
        for row in csv.DictReader(stream):
            unique[int(row["stamp_ns"])] = [float(row[k]) for k in ["x", "y", "z", "qx", "qy", "qz", "qw"]]
    times = np.array(sorted(unique), dtype=np.int64)
    poses = np.array([unique[int(t)] for t in times])
    if len(times) < 2 or not np.all(np.isfinite(poses)):
        raise ValueError("再計算poseが不足または非有限")
    path = []
    for stamp, pose in zip(times, poses):
        if not path or np.linalg.norm(pose[:2]-np.array(path[-1][1:3])) >= .25:
            path.append(pose_for_footprint(stamp, pose))
    # nsをfloatに変換する前に整数時刻列を退避する（float64ではnsの下位桁が失われる）。
    path_times = np.array([v[0] for v in path], dtype=np.int64)
    path = np.array(path, dtype=np.float64)
    frame_rows, distance_best, diagnostics = [], {}, []
    persistent_grid = (RollingElevationGrid(params["map_size_x"], params["map_size_y"], params["resolution"])
                       if args.temporal_fusion else None)
    rclpy.init()
    seen_trace_stamps = []
    try:
        buffer = Buffer(); infos = []
        for _, channel, record in (messages(args.bag, ["/tf_static", params["camera_info_topic"]])
                                   if trace_fusions is None else []):
            if channel.topic == "/tf_static":
                for tf in deserialize_message(record.data, TFMessage).transforms:
                    buffer.set_transform_static(tf, "offline_bag")
            else:
                info = deserialize_message(record.data, CameraInfo)
                infos.append((stamp_ns(info), info))
        counts = dict(depth_frames=0, pose_unavailable=0, zero_valid_pixels=0, evaluated_frames=0,
                      obstacle_valid_cells=0, trace_not_adopted=0)
        for _, _, record in messages(args.bag, [params["depth_topic"]]):
            depth = deserialize_message(record.data, Image)
            stamp = stamp_ns(depth); counts["depth_frames"] += 1
            used = None if trace_fusions is None else trace_fusions.get(stamp)
            if trace_fusions is not None and used is None:
                counts["trace_not_adopted"] += 1; continue
            if used is not None:
                if (used["map_frame"] != "odom" or used["width"] != depth.width
                        or used["height"] != depth.height or used["image_frame"] != depth.header.frame_id):
                    raise ValueError("traceのmap frameまたは画像寸法が不一致")
                seen_trace_stamps.append(stamp)
                pose = np.array(used["base_to_map"]["translation"]+used["base_to_map"]["quaternion"])
            else:
                pose = interpolate_pose(times, poses, stamp, args.max_pose_gap)
            if pose is None:
                counts["pose_unavailable"] += 1; continue
            if used is not None:
                intrinsics = used["intrinsics"]
            elif infos:
                info = min(infos, key=lambda v: abs(v[0]-stamp))[1]
                if info.width != depth.width or info.height != depth.height:
                    raise ValueError("camera_info解像度が不一致")
                intrinsics = [info.k[0], info.k[4], info.k[2], info.k[5]]
            else:
                if not params.get("allow_fallback_intrinsics") or depth.width != params["fallback_width"] or depth.height != params["fallback_height"]:
                    raise ValueError("利用可能なintrinsicsがありません")
                intrinsics = [params["fallback_"+k] for k in ["fx", "fy", "cx", "cy"]]
            optical = sampled_points(depth, *intrinsics, args.stride, params["min_depth"], params["max_depth"])
            if not len(optical):
                counts["zero_valid_pixels"] += 1; continue
            if used is not None:
                # camera→odomそのものを使う。camera→baseとposeを再合成して置き換えない。
                points = transform_points(optical, recorded_transform(used["camera_to_map"]))
                camera_world = np.array(used["camera_to_map"]["translation"])
            else:
                transform = buffer.lookup_transform("base_link", args.camera_frame, Time(nanoseconds=stamp))
                base = transform_points(optical, transform.transform)
                rotation = quaternion_matrix(quaternion(pose[3:]))
                points = base @ rotation.T + pose[:3]
                translation = transform.transform.translation
                camera_world = rotation @ np.array([translation.x, translation.y, translation.z]) + pose[:3]
            frame_pose = pose_for_footprint(stamp, pose)
            arrays = evaluate_points(
                points, params, stamp, center=pose[:2], heading_yaw=frame_pose[-1],
                relative_elevation_offset=params["nominal_camera_height_above_ground"]-camera_world[2],
                grid=persistent_grid,
                observation_variance=params["measurement_variance"]+params["depth_variance_per_meter_sq"]*np.sum(optical*optical, axis=1),
            )
            counts["evaluated_frames"] += 1
            arrays["selected_hazard"] = arrays["reference_hazard"] if args.single_frame_obstacle else arrays["hazard"]
            counts["obstacle_valid_cells"] += int(np.isfinite(arrays["obstacle_height"]).sum())
            # 将来通過する経路位置との対応。従来と同じ約2 m・±.25 m・前方・12秒内。
            dx, dy = path[:, 1]-pose[0], path[:, 2]-pose[1]
            distance = np.hypot(dx, dy)
            forward = dx*np.cos(frame_pose[-1])+dy*np.sin(frame_pose[-1])
            eligible = np.flatnonzero((path_times > stamp+200_000_000) & (path_times-stamp <= 12_000_000_000)
                                     & (np.abs(distance-2) <= .25) & (forward > 0))
            rows = []
            for index in eligible:
                row = dict(frame_stamp_ns=stamp, passage_stamp_ns=int(path_times[index]),
                           x=float(path[index, 1]), y=float(path[index, 2]), lead_distance=float(distance[index]))
                row.update(evaluate_footprint(arrays, path[index], args.width, args.length, params,
                                              cell_diagnostics=args.footprint_cell_diagnostics))
                rows.append((index, row))
                # 距離ベース：各経路sampleで最良距離・同率なら最新frameを一つだけ採用。
                score = (abs(distance[index]-2), -stamp)
                if index not in distance_best or score < distance_best[index][0]:
                    distance_best[index] = (score, row)
            if rows:
                # frameベース：一つのframeで経路を重複カウントしない。2m誤差最小を選ぶ。
                index, row = min(rows, key=lambda item: (abs(item[1]["lead_distance"]-2), item[1]["passage_stamp_ns"]))
                frame_rows.append(row)
                if len(frame_rows) in [1, 500, 1000]:
                    save_example(output, arrays, depth, stamp)
            diagnostics.append(dict(frame_stamp_ns=stamp, pixels=len(optical),
                                    observed_cells=int((arrays["pixel_count"]>0).sum()),
                                    hazard_known=int(np.isfinite(arrays["hazard"]).sum()),
                                    hazard_black=int(np.count_nonzero(arrays["hazard"]>=1)),
                                    path_matches=len(rows)))
            if counts["evaluated_frames"] % 200 == 0:
                print("evaluated %d frames" % counts["evaluated_frames"], flush=True)
        if trace_fusions is not None and seen_trace_stamps != list(trace_fusions):
            raise ValueError("元bagとtraceの画像stamp/順序が一致しません。別bagの取り違えを確認してください")
        distance_rows = [distance_best[index][1] for index in sorted(distance_best)]
        summary = dict(counts=counts, parameters=params, stride=args.stride,
                       intrinsics_source="mapper trace" if trace_fusions is not None else ("bag camera_info" if infos else "YAML fallback"),
                       mapper_trace=args.mapper_trace, projection_pose_source=("actual mapper TF" if trace_fusions is not None else "interpolated pose CSV"),
                       trace_snapshot_count=len(trace_snapshots), evaluation_schedule="each adopted image",
                       pose_source=str(Path(args.poses_csv).resolve()), max_pose_gap_seconds=args.max_pose_gap,
                       temporal_fusion=args.temporal_fusion, footprint_width=args.width, footprint_length=args.length,
                       footprint_cell_diagnostics=args.footprint_cell_diagnostics,
                       single_frame_obstacle=args.single_frame_obstacle,
                       primary_metric="reference_hazard" if args.single_frame_obstacle else "hazard",
                       path_samples_total=len(path), path_samples_unmatched=len(path)-len(distance_rows),
                       frames_without_path_match=counts["evaluated_frames"]-len(frame_rows),
                       frame_weighted=summarize(frame_rows), distance_weighted=summarize(distance_rows))
        # フラグで選択した主評価を明示し、従来confidence条件の率と取り違えないようにする。
        summary["selected_frame_weighted"] = summary["frame_weighted"][summary["primary_metric"]]
        summary["selected_distance_weighted"] = summary["distance_weighted"][summary["primary_metric"]]
        (output/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
        write_csv(output/"frame_footprints.csv", frame_rows)
        write_csv(output/"path_footprints.csv", distance_rows)
        write_csv(output/"frame_diagnostics.csv", diagnostics)
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
