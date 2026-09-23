#!/usr/bin/env python3
"""再計算したodom経路で、通過前のterrain hazardを評価する。

ROS replay中に`/odometry/local`とhazard/componentのOccupancyGridを保持し、replay終了後に
`--output`へJSONとCSVを保存する。通過地点の事後地図を使うと「通過したから安全」という
情報漏れになるため、約2 m手前でpublish済みの地図だけを参照する。
"""

import argparse
import csv
import json
import math
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry


LAYERS = ("terrain_hazard_debug", "slope_debug", "roughness_debug",
          "step_height_debug", "obstacle_height_debug")
DIAGNOSTIC_LAYERS = ("relative_elevation_debug", "elevation_variance_debug",
                     "observation_count_debug", "observation_age_debug")


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Collector:
    def __init__(self):
        self.node = rclpy.create_node("traversed_terrain_evaluator")
        self.poses = []
        self.frames = {}
        self.last_input = time.monotonic()
        self.odom_count = 0
        self.node.create_subscription(Odometry, "/odometry/local", self.on_odom, 100)
        for name in LAYERS + DIAGNOSTIC_LAYERS:
            topic = "/depth_elevation_mapper/" + name
            self.node.create_subscription(
                OccupancyGrid, topic,
                lambda msg, layer=name: self.on_grid(layer, msg), 20,
            )

    def on_odom(self, msg):
        # 最新localizerのodom poseだけを収集する。bag内の古いodomはreplayしない。
        if msg.header.frame_id != "odom":
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                          1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0,
                          2 * (q.w * q.y - q.z * q.x))))
        self.poses.append((stamp_ns(msg.header.stamp), p.x, p.y, p.z,
                           roll, pitch, yaw_of(q)))
        self.odom_count += 1
        self.last_input = time.monotonic()

    def on_grid(self, layer, msg):
        if msg.header.frame_id != "odom":
            return
        key = stamp_ns(msg.header.stamp)
        frame = self.frames.setdefault(key, {})
        # int8への変換は1 mapあたり一度だけ。ROS message objectを保持しない。
        frame[layer] = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        ).copy()
        if layer == LAYERS[0]:
            frame["origin"] = (msg.info.origin.position.x,
                               msg.info.origin.position.y)
            frame["resolution"] = msg.info.resolution
        self.last_input = time.monotonic()


def footprint_values(layer, origin, resolution, pose, length, width):
    """通過姿勢における長方形footprint内のgrid cell値を抽出する。"""
    _, x, y, _, _, _, yaw = pose
    # cell中心が車体footprintに入るものを列挙する。bounding boxは小さいため計算量は軽い。
    radius = math.hypot(length, width) / 2
    ix0 = max(0, math.floor((x - radius - origin[0]) / resolution))
    ix1 = min(layer.shape[1], math.ceil((x + radius - origin[0]) / resolution))
    iy0 = max(0, math.floor((y - radius - origin[1]) / resolution))
    iy1 = min(layer.shape[0], math.ceil((y + radius - origin[1]) / resolution))
    if ix0 >= ix1 or iy0 >= iy1:
        return np.empty(0, dtype=np.int8)
    yy, xx = np.mgrid[iy0:iy1, ix0:ix1]
    dx = origin[0] + (xx + 0.5) * resolution - x
    dy = origin[1] + (yy + 0.5) * resolution - y
    forward = dx * math.cos(yaw) + dy * math.sin(yaw)
    lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
    inside = (np.abs(forward) <= length / 2) & (np.abs(lateral) <= width / 2)
    return layer[iy0:iy1, ix0:ix1][inside]


def analyse(collector, target_distance, distance_tolerance, width, length):
    """約2 m前の地図を選び、走行軌跡を約0.25 m間隔で一度ずつ判定する。"""
    poses = sorted(collector.poses)
    times = np.asarray([p[0] for p in poses], dtype=np.int64)
    frame_times = np.asarray(sorted(k for k, v in collector.frames.items()
                                    if LAYERS[0] in v), dtype=np.int64)
    rows = []
    if not poses or not len(frame_times):
        return rows, {"error": "odomまたはhazard mapがありません"}
    last_selected = None
    for pose in poses:
        t, x, y, z, roll, pitch, yaw = pose
        if last_selected is not None and math.hypot(
                x - last_selected[1], y - last_selected[2]) < 0.25:
            continue
        last_selected = pose
        # 各通過姿勢について、それより前に車体が約2 m離れていたmapを探す。
        # 近い候補が複数あれば距離誤差が最小で、同率なら新しいmapを採用する。
        stop = int(np.searchsorted(frame_times, t - 200_000_000))
        candidates = frame_times[max(0, stop - 40):stop]
        best = None
        for ft in candidates:
            # 地図時刻より後のodomを使うと、わずかでも未来の自己位置が混入する。
            # 直前のodom sampleを選び、予測時点の車体位置を再現する。
            prior_idx = int(np.searchsorted(times, ft, side="right")) - 1
            if prior_idx < 0 or t - ft > 12_000_000_000:
                continue
            prior = poses[prior_idx]
            dx, dy = x - prior[1], y - prior[2]
            distance = math.hypot(dx, dy)
            if abs(distance - target_distance) > distance_tolerance:
                continue
            # 後退や横滑り時は前方FOVの予測評価から外す。
            if dx * math.cos(prior[6]) + dy * math.sin(prior[6]) <= 0:
                continue
            score = (abs(distance - target_distance), -int(ft))
            if best is None or score < best[0]:
                best = (score, int(ft), distance)
        row = {"passage_time_ns": int(t), "x_odom_m": x, "y_odom_m": y,
               "odom_z_m": z, "odom_roll_deg": math.degrees(roll),
               "odom_pitch_deg": math.degrees(pitch),
               "map_odom_z_m": "", "map_odom_roll_deg": "",
               "map_odom_pitch_deg": "",
               "lookahead_map_time_ns": "", "lead_distance_m": "",
               "center_hazard": "", "footprint_cells": 0,
               "observed_cells": 0, "black_cells": 0,
               "black_fraction_observed": "", "black_any": "",
               "slope_black": "", "roughness_black": "",
               "step_black": "", "obstacle_black": "",
               "center_slope_deg": "", "center_roughness_m": "",
               "center_step_m": "", "center_obstacle_m": "",
               "center_relative_elevation_m": "", "center_variance_m2": "",
               "center_observation_count": "", "center_age_s": "",
               "local_elevation_range_m": "", "local_elevation_std_m": "",
               "local_variance_max_m2": "", "local_count_min": "",
               "local_age_max_s": ""}
        if best is not None:
            _, ft, distance = best
            prior_idx = int(np.searchsorted(times, ft, side="right")) - 1
            map_pose = poses[prior_idx]
            row.update({"map_odom_z_m": map_pose[3],
                        "map_odom_roll_deg": math.degrees(map_pose[4]),
                        "map_odom_pitch_deg": math.degrees(map_pose[5])})
            frame = collector.frames[ft]
            layer = frame[LAYERS[0]]
            origin, resolution = frame["origin"], frame["resolution"]
            ix = math.floor((x - origin[0]) / resolution)
            iy = math.floor((y - origin[1]) / resolution)
            center = int(layer[iy, ix]) if 0 <= iy < layer.shape[0] and 0 <= ix < layer.shape[1] else -1
            values = footprint_values(layer, origin, resolution, pose, length, width)
            known = values >= 0
            black = values >= 100
            row.update({"lookahead_map_time_ns": ft, "lead_distance_m": distance,
                        "center_hazard": center, "footprint_cells": len(values),
                        "observed_cells": int(known.sum()),
                        "black_cells": int(black.sum()),
                        "black_fraction_observed": float(black.sum() / known.sum())
                        if known.any() else "",
                        "black_any": int(black.any()) if known.any() else ""})
            if center >= 0:
                # OccupancyGridの表示値を既知の線形rangeへ戻す。0..100量子化なので、
                # これは物理値の約1%刻みの診断値である。
                for name, column, maximum in (
                    ("slope_debug", "center_slope_deg", 20.0),
                    ("roughness_debug", "center_roughness_m", 0.03),
                    ("step_height_debug", "center_step_m", 0.07),
                    ("obstacle_height_debug", "center_obstacle_m", 0.20),
                ):
                    q = int(frame[name][iy, ix]) if name in frame else -1
                    row[column] = q * maximum / 100.0 if q >= 0 else ""
                for name, column, low, high in (
                    ("relative_elevation_debug", "center_relative_elevation_m", -0.30, 0.30),
                    ("elevation_variance_debug", "center_variance_m2", 0.0, 0.02),
                    ("observation_count_debug", "center_observation_count", 0.0, 10.0),
                    ("observation_age_debug", "center_age_s", 0.0, 5.0),
                ):
                    q = int(frame[name][iy, ix]) if name in frame else -1
                    row[column] = low + q * (high - low) / 100.0 if q >= 0 else ""

                # 局所3x3 relative elevationから、平面推定に入力された近傍の散らばりを
                # 要約する。未知セルは統計から除外し、unknownを低い高さと誤認しない。
                elev = frame.get("relative_elevation_debug")
                if elev is not None:
                    patch = elev[max(0, iy - 1):min(elev.shape[0], iy + 2),
                                 max(0, ix - 1):min(elev.shape[1], ix + 2)]
                    valid_patch = patch >= 0
                    if np.any(valid_patch):
                        physical = -0.30 + patch[valid_patch].astype(np.float32) * 0.006
                        row["local_elevation_range_m"] = float(np.ptp(physical))
                        row["local_elevation_std_m"] = float(np.std(physical))
                for name, column, maximum in (
                    ("elevation_variance_debug", "local_variance_max_m2", 0.02),
                    ("observation_count_debug", "local_count_min", 10.0),
                    ("observation_age_debug", "local_age_max_s", 5.0),
                ):
                    diagnostic = frame.get(name)
                    if diagnostic is not None:
                        patch = diagnostic[max(0, iy - 1):min(diagnostic.shape[0], iy + 2),
                                           max(0, ix - 1):min(diagnostic.shape[1], ix + 2)]
                        valid_patch = patch >= 0
                        if np.any(valid_patch):
                            physical = patch[valid_patch].astype(np.float32) * maximum / 100.0
                            row[column] = (float(np.max(physical)) if "max" in column or "age" in column
                                           else float(np.min(physical)))
            for name, key in zip(LAYERS[1:], ("slope_black", "roughness_black",
                                               "step_black", "obstacle_black")):
                if name in frame:
                    cue = footprint_values(frame[name], origin, resolution,
                                           pose, length, width)
                    row[key] = int((cue >= 100).any()) if known.any() else ""
        rows.append(row)
    observed = [r for r in rows if r["observed_cells"] > 0]
    predicted = [r for r in rows if r["lookahead_map_time_ns"] != ""]
    full = [r for r in observed if r["observed_cells"] == r["footprint_cells"]]
    summary = {
        "bag_odom_samples": collector.odom_count,
        "hazard_map_frames": len(frame_times),
        "path_samples_0_25m": len(rows),
        "lookahead_candidates": len(predicted),
        "with_observed_footprint": len(observed),
        "fully_observed_footprint": len(full),
        "black_any_observed_footprint": sum(r["black_any"] for r in observed),
        "black_any_fully_observed_footprint": sum(r["black_any"] for r in full),
        "center_observed": sum(r["center_hazard"] != -1 for r in predicted),
        "center_black": sum(r["center_hazard"] == 100 for r in predicted),
        "observed_cell_count": sum(r["observed_cells"] for r in observed),
        "black_cell_count": sum(r["black_cells"] for r in observed),
        "footprint_width_m": width, "footprint_length_m": length,
        "lookahead_distance_m": target_distance,
        "lookahead_tolerance_m": distance_tolerance,
    }
    for cue in ("slope_black", "roughness_black", "step_black", "obstacle_black"):
        summary[cue + "_any"] = sum(r[cue] == 1 for r in observed)
    return rows, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=245.0)
    parser.add_argument("--lead-distance", type=float, default=2.0)
    parser.add_argument("--lead-tolerance", type=float, default=0.25)
    parser.add_argument("--width", type=float, default=0.45)
    parser.add_argument("--length", type=float, default=0.55)
    args = parser.parse_args()
    rclpy.init()
    collector = Collector()
    stop = False

    def request_stop(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    started = time.monotonic()
    try:
        while not stop and time.monotonic() - started < args.duration:
            rclpy.spin_once(collector.node, timeout_sec=0.2)
    finally:
        rows, summary = analyse(collector, args.lead_distance,
                                args.lead_tolerance, args.width, args.length)
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        with (args.output / "path_samples.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else [])
            if rows:
                writer.writeheader()
                writer.writerows(rows)
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
        collector.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
