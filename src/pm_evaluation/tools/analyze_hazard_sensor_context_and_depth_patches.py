#!/usr/bin/env python3
"""黒判定追跡frameのIMU/速度文脈とdepth近傍品質を一括診断する。

保存済みの舗装特徴pair（黒地点5/12/13）のsource stampだけを基準に、bag内の
Wit/OAK IMU、wheel/VIO odometry、対応するdepth画像を照合する。depthは追跡pixelの
3x3/5x5/9x9近傍で有効率・散らばり・中央値を測る。bag走査中は全depth messageから
header stampを読む必要があるが、pixel配列として保持するのは対象6 frameだけに限る。

この解析は相関と入力品質の検査で、独立ground truthではない。raw VIO/wheel値は
replay localizerの最終姿勢そのものではなく、動的状態を説明するための補助signal。
"""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np


TOPIC_TYPES = {
    "/oak/depth/image_raw": "image",
    "/oak/imu/data": "imu",
    "/wit/imu": "imu",
    "/wheel/odometry": "odom",
    "/vio/odometry": "odom",
}


def stamp_ns(message):
    """ROS message header stampをnanosecond整数へ変換する。"""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def percentile(values, quantile):
    """空配列を安全に扱うpercentile helper。"""
    return float(np.percentile(values, quantile)) if values else None


def read_tracks(path):
    """舗装特徴追跡結果から対象pairと各featureを取得する。"""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    selected = [event for event in data["events"]
                if event["target_index"] in (5, 12, 13) and event.get("matches")]
    if len(selected) != 3:
        raise RuntimeError("黒地点5/12/13の追跡結果3件が必要です")
    stamps = sorted({stamp for event in selected for stamp in
                     (event["source_stamp0_ns"], event["source_stamp1_ns"])})
    return selected, stamps


def read_frame_pose(path):
    """frame event CSVからsource stamp別の姿勢/位置を一つずつ読む。"""
    rows_by_stamp = {}
    with Path(path).open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            stamp = int(row["source_stamp_ns"])
            rows_by_stamp.setdefault(stamp, row)
    return rows_by_stamp


def read_map_support_pixels(path):
    """色投影診断summaryからmapに実際に使われたsource pixelを読む。"""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return {target["index"]: target.get("images", []) for target in data["targets"]}


def read_exact_support_updates(forensic_dir, depth_quality_rows):
    """実使用pixelをraw投影CSVと同stamp・同cellのfusion記録へ結ぶ。

    RGB特徴ではなく、前段で特定したmap support pixelの(u,v)をキーにする。
    frame_pixels.csvから同じsource stamp・同じ画素が割り当てられたodom cellと
    world座標を得て、frame_events.csvの同stamp・同cellからfusion前後を読む。
    stampだけ、あるいは画素だけで結合すると別フレーム/別セルを誤結合するため、
    三つの条件が一致する行だけを採用する。
    """
    pixel_path = forensic_dir / "frame_pixels.csv"
    event_path = forensic_dir / "frame_events.csv"
    if not pixel_path.is_file() or not event_path.is_file():
        raise FileNotFoundError("forensic CSVがありません: {}".format(forensic_dir))

    # 六つの対象frameだけを結合対象にし、大きなCSVをメモリへ複製しない。
    wanted = {}
    for target in depth_quality_rows:
        for frame in target["frames"]:
            support = frame.get("map_support_pixel")
            if frame.get("image_found") and support:
                key = (int(frame["stamp_ns"]), int(support["depth_pixel"][0]),
                       int(support["depth_pixel"][1]))
                wanted[key] = {"target_index": target["target_index"],
                               "frame": frame["frame"], "support": support}

    cells = {}
    with pixel_path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            key = (int(row["source_stamp_ns"]), int(row["pixel_u"]),
                   int(row["pixel_v"]))
            if key in wanted:
                cell = (int(row["odom_cell_x"]), int(row["odom_cell_y"]))
                cells.setdefault(key, {})[cell] = {
                    "odom_cell_x": cell[0], "odom_cell_y": cell[1],
                    "world_x_m": float(row["world_x_m"]),
                    "world_y_m": float(row["world_y_m"]),
                    "world_z_m": float(row["world_z_m"]),
                    "axial_depth_m": float(row["axial_depth_m"]),
                }

    wanted_event_keys = {
        (stamp, cell[0], cell[1])
        for (stamp, _, _), cell_rows in cells.items()
        for cell in cell_rows
    }
    event_rows = {}
    with event_path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            key = (int(row["source_stamp_ns"]), int(row["odom_cell_x"]),
                   int(row["odom_cell_y"]))
            if key in wanted_event_keys:
                # 対象pixelから得た同stamp・同cell行だけ保持する。
                event_rows[key] = row

    results = []
    for key, metadata in sorted(wanted.items()):
        projected = []
        for cell_key, point in sorted(cells.get(key, {}).items()):
            event = event_rows.get((key[0], cell_key[0], cell_key[1]))
            record = dict(point)
            if event is None:
                record["fusion_event_found"] = False
            else:
                record.update({
                    "fusion_event_found": True,
                    "frame_sample_count": int(event["sample_count"]),
                    "frame_min_world_z_m": float(event["frame_min_world_z_m"]),
                    "frame_max_world_z_m": float(event["frame_max_world_z_m"]),
                    "fusion_mode": int(event["fusion_mode"]),
                    "ground_before_fusion_m": float(event["ground_before_m"]),
                    "ground_after_fusion_m": float(event["ground_after_m"]),
                    "relative_before_m": float(event["relative_before_m"]),
                    "relative_after_m": float(event["relative_after_m"]),
                    "base_z_m": float(event["base_z_m"]),
                    "base_roll_rad": float(event["base_roll_rad"]),
                    "base_pitch_rad": float(event["base_pitch_rad"]),
                })
            projected.append(record)
        results.append({
            "target_index": metadata["target_index"], "frame": metadata["frame"],
            "source_stamp_ns": key[0], "support_pixel": [key[1], key[2]],
            "projected_cells": projected,
        })
    return results


def extract_bag_data(bag_dir, target_stamps, margin_s):
    """必要な6 depth imageと対象時間窓のIMU/odometryだけMCAPから抽出する。"""
    import rclpy
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, Imu

    earliest = min(target_stamps) - int(margin_s * 1e9)
    latest = max(target_stamps) + int(margin_s * 1e9)
    by_topic = {topic: [] for topic in TOPIC_TYPES}
    images = {}
    type_by_name = {"image": Image, "imu": Imu, "odom": Odometry}
    rclpy.init()
    try:
        for mcap_path in sorted(Path(bag_dir).glob("*.mcap")):
            with mcap_path.open("rb") as stream:
                reader = make_reader(stream)
                for _, channel, record in reader.iter_messages(topics=list(TOPIC_TYPES)):
                    topic = channel.topic if hasattr(channel, "topic") else channel
                    kind = TOPIC_TYPES[topic]
                    msg = deserialize_message(record.data, type_by_name[kind])
                    stamp = stamp_ns(msg)
                    if kind == "image":
                        # header stampを確認するためbag内depthは全てdeserializeするが、
                        # pixel配列を保持するのは6個の追跡stampと一致したframeだけ。
                        if stamp in target_stamps:
                            if msg.encoding not in ("16UC1", "mono16"):
                                raise RuntimeError("予期しないdepth encoding: " + msg.encoding)
                            dtype = ">u2" if msg.is_bigendian else "<u2"
                            words_per_row = msg.step // 2
                            raw = np.frombuffer(msg.data, dtype=dtype).reshape(
                                msg.height, words_per_row)[:, :msg.width]
                            images[stamp] = raw.astype(np.float32) * 0.001
                    elif earliest <= stamp <= latest:
                        by_topic[topic].append((stamp, msg))
    finally:
        rclpy.shutdown()
    for topic in by_topic:
        by_topic[topic].sort(key=lambda item: item[0])
    return by_topic, images


def nearest(entries, stamp_ns, max_delta_ms=30.0):
    """指定stampに近いsampleと符号付き時間差を返す。"""
    stamps = [item[0] for item in entries]
    index = bisect.bisect_left(stamps, stamp_ns)
    candidates = [i for i in (index - 1, index) if 0 <= i < len(stamps)]
    if not candidates:
        return None
    chosen = min(candidates, key=lambda i: abs(stamps[i] - stamp_ns))
    delta_ms = (stamps[chosen] - stamp_ns) / 1e6
    return (entries[chosen][1], delta_ms) if abs(delta_ms) <= max_delta_ms else None


def odom_values(msg):
    """Odometryから速度・鉛直位置と親/子frameを取り出す。"""
    twist = msg.twist.twist
    pose = msg.pose.pose
    return {
        "frame_id": msg.header.frame_id, "child_frame_id": msg.child_frame_id,
        "linear_xyz_mps": [twist.linear.x, twist.linear.y, twist.linear.z],
        "linear_speed_xy_mps": math.hypot(twist.linear.x, twist.linear.y),
        "angular_xyz_rps": [twist.angular.x, twist.angular.y, twist.angular.z],
        "position_z_m": pose.position.z,
    }


def imu_values(msg):
    """IMUから姿勢、加速度、gyroを取り出す。"""
    q = msg.orientation
    roll = math.degrees(math.atan2(2 * (q.w*q.x + q.y*q.z),
                                   1 - 2 * (q.x*q.x + q.y*q.y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0,
                  2 * (q.w*q.y - q.z*q.x)))))
    accel = [msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z]
    gyro = [msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z]
    return {
        "frame_id": msg.header.frame_id, "roll_deg": roll, "pitch_deg": pitch,
        "orientation_covariance_0": msg.orientation_covariance[0],
        "accel_xyz_mps2": accel, "accel_norm_mps2": float(np.linalg.norm(accel)),
        "gyro_xyz_rps": gyro, "gyro_norm_rps": float(np.linalg.norm(gyro)),
    }


def patch_statistics(depth, u, v, radius):
    """depth画素周囲の正値かつmapper範囲内のdepth分布を要約する。"""
    center_x, center_y = int(round(u)), int(round(v))
    x0, x1 = max(0, center_x - radius), min(depth.shape[1], center_x + radius + 1)
    y0, y1 = max(0, center_y - radius), min(depth.shape[0], center_y + radius + 1)
    patch = depth[y0:y1, x0:x1]
    valid = patch[(patch >= 0.4) & (patch <= 5.0)]
    expected_area = (2 * radius + 1) ** 2
    center = (float(depth[center_y, center_x]) if 0 <= center_x < depth.shape[1]
              and 0 <= center_y < depth.shape[0] else 0.0)
    return {
        "radius_px": radius, "sample_count": int(patch.size),
        "valid_count": int(valid.size),
        "valid_fraction": float(valid.size / expected_area),
        "center_depth_m": center if center > 0 else None,
        "median_depth_m": float(np.median(valid)) if valid.size else None,
        "p10_p90_spread_m": (float(np.percentile(valid, 90) - np.percentile(valid, 10))
                             if valid.size else None),
        "min_max_spread_m": float(np.max(valid) - np.min(valid)) if valid.size else None,
        "std_m": float(np.std(valid)) if valid.size else None,
    }


def patch_matrix(depth, u, v, radius=1):
    """小窓のraw depth値をm単位で返す。範囲外・無効値は0とする。"""
    center_x, center_y = int(round(u)), int(round(v))
    x0, x1 = max(0, center_x - radius), min(depth.shape[1], center_x + radius + 1)
    y0, y1 = max(0, center_y - radius), min(depth.shape[0], center_y + radius + 1)
    patch = depth[y0:y1, x0:x1].copy()
    patch[(patch < 0.4) | (patch > 5.0)] = 0.0
    return np.round(patch, 3).tolist()


def depth_quality(events, images, support_by_target, pose_by_stamp):
    """全追跡点の3x3/5x5/9x9局所depth品質をevent/frame単位でまとめる。"""
    output = []
    for event in events:
        frames = []
        for suffix, stamp_key, pixel_key, depth_key in (
                ("0", "source_stamp0_ns", "depth_pixel0", "depth_m0"),
                ("1", "source_stamp1_ns", "depth_pixel1", "depth_m1")):
            stamp = event[stamp_key]
            depth = images.get(stamp)
            if depth is None:
                frames.append({"frame": suffix, "stamp_ns": stamp, "image_found": False})
                continue
            feature_rows = []
            for match in event["matches"]:
                u, v = match[pixel_key]
                stats = {str(2 * radius + 1): patch_statistics(depth, u, v, radius)
                         for radius in (1, 2, 4)}
                feature_rows.append({
                    "feature_id": match["feature_id"], "depth_pixel": [u, v],
                    "track_depth_m": match[depth_key],
                    "track_spread_m": match["depth_patch_spread_m" + suffix],
                    "forward_backward_error_px": match["forward_backward_error_px"],
                    "patches": stats,
                    "patch3x3_m": patch_matrix(depth, u, v),
                })
            p3 = [row["patches"]["3"]["p10_p90_spread_m"] for row in feature_rows
                  if row["patches"]["3"]["p10_p90_spread_m"] is not None]
            p9 = [row["patches"]["9"]["p10_p90_spread_m"] for row in feature_rows
                  if row["patches"]["9"]["p10_p90_spread_m"] is not None]
            valid_3 = [row["patches"]["3"]["valid_fraction"] for row in feature_rows]
            support_rows = [row for row in support_by_target.get(event["target_index"], [])
                            if int(row["source_stamp_ns"]) == stamp]
            map_support = None
            if support_rows:
                support = support_rows[0]
                su, sv = support["depth_pixel"]
                nearest_track_px = min((math.hypot(float(su) - row["depth_pixel"][0],
                                                  float(sv) - row["depth_pixel"][1])
                                        for row in feature_rows), default=None)
                map_support = {
                    "depth_pixel": [su, sv], "depth_m_recorded": support["depth_m"],
                    "map_relative_elevation_m": support["support_map_elevation_m"],
                    "nearest_tracked_feature_distance_px": nearest_track_px,
                    "patches": {str(2 * radius + 1): patch_statistics(
                        depth, su, sv, radius) for radius in (1, 2, 4)},
                    "patch3x3_m": patch_matrix(depth, su, sv),
                }
                # 周辺depthの差が、単なる視線方向のrange勾配か、world zの段差かを区別する。
                pose_row = pose_by_stamp[stamp]
                q = np.array([float(pose_row[key]) for key in
                              ("camera_qx", "camera_qy", "camera_qz", "camera_qw")])
                xq, yq, zq, wq = q / np.linalg.norm(q)
                rotation = np.array([
                    [1 - 2*(yq*yq + zq*zq), 2*(xq*yq - zq*wq), 2*(xq*zq + yq*wq)],
                    [2*(xq*yq + zq*wq), 1 - 2*(xq*xq + zq*zq), 2*(yq*zq - xq*wq)],
                    [2*(xq*zq - yq*wq), 2*(yq*zq + xq*wq), 1 - 2*(xq*xq + yq*yq)],
                ])
                translation = np.array([float(pose_row[key]) for key in
                                        ("camera_x_m", "camera_y_m", "camera_z_m")])
                center_z_world = None
                support_z_neighbors = []
                support_xyz_neighbors = []
                fx, fy, cx, cy = 574.28826904, 574.28826904, 354.75085449, 215.23262024
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        px, py = int(round(su)) + dx, int(round(sv)) + dy
                        value = float(depth[py, px])
                        if not 0.4 <= value <= 5.0:
                            support_z_neighbors.append(None)
                            support_xyz_neighbors.append(None)
                            continue
                        point = np.array([(px - cx) * value / fx,
                                          (py - cy) * value / fy, value])
                        world_point = rotation @ point + translation
                        world_z = float(world_point[2])
                        support_xyz_neighbors.append(world_point)
                        if dx == 0 and dy == 0:
                            center_z_world = world_z
                        support_z_neighbors.append(world_z)
                finite_neighbor_z = [value for value in support_z_neighbors if value is not None]
                deltas = ([abs(value - center_z_world) for value in finite_neighbor_z]
                          if center_z_world is not None else [])
                map_support["center_world_z_m_from_own_pose"] = center_z_world
                map_support["neighbor_world_z_abs_delta_m"] = {
                    "count": len(deltas), "p50": percentile(deltas, 50),
                    "p90": percentile(deltas, 90), "max": max(deltas) if deltas else None,
                }
                map_support["neighbor_world_z_3x3_m"] = [
                    [round(support_z_neighbors[3 * iy + ix], 4)
                     if support_z_neighbors[3 * iy + ix] is not None else None
                     for ix in range(3)] for iy in range(3)]
                center_xyz = support_xyz_neighbors[4]
                xy_offsets = ([float(np.linalg.norm(value[:2] - center_xyz[:2]))
                              for value in support_xyz_neighbors if value is not None]
                             if center_xyz is not None else [])
                plane_points = np.asarray([value for value in support_xyz_neighbors
                                           if value is not None])
                plane_result = None
                if len(plane_points) >= 6:
                    design = np.column_stack((plane_points[:, 0], plane_points[:, 1],
                                              np.ones(len(plane_points))))
                    coefficients, _, _, _ = np.linalg.lstsq(
                        design, plane_points[:, 2], rcond=None)
                    residual = np.abs(plane_points[:, 2] - design @ coefficients)
                    plane_result = {
                        "slope_deg": math.degrees(math.atan(math.hypot(
                            float(coefficients[0]), float(coefficients[1])))),
                        "residual_median_m": float(np.median(residual)),
                        "residual_max_m": float(np.max(residual)),
                        "sample_count": int(len(plane_points)),
                    }
                map_support["neighbor_xy_distance_from_center_m"] = {
                    "max": max(xy_offsets) if xy_offsets else None,
                    "median": percentile(xy_offsets, 50),
                }
                map_support["least_squares_patch_plane"] = plane_result
            frames.append({
                "frame": suffix, "stamp_ns": stamp, "image_found": True,
                "encoding": "16UC1 interpreted millimeters",
                "feature_count": len(feature_rows),
                "p3x3_p10_p90_spread_m": {
                    "p50": percentile(p3, 50), "p90": percentile(p3, 90),
                    "max": max(p3) if p3 else None},
                "p9x9_p10_p90_spread_m": {
                    "p50": percentile(p9, 50), "p90": percentile(p9, 90),
                    "max": max(p9) if p9 else None},
                "valid_fraction_3x3_p50": percentile(valid_3, 50),
                "map_support_pixel": map_support,
                "features": feature_rows,
            })
        output.append({"target_index": event["target_index"], "frames": frames})
    return output


def sensor_context(events, stamps, data_by_topic):
    """各特徴pair端でIMU・wheel/VIO sampleの最近傍値と時差を記録する。"""
    outputs = []
    for event in events:
        pair = []
        for frame, stamp in enumerate((event["source_stamp0_ns"],
                                       event["source_stamp1_ns"])):
            item = {"frame": frame, "stamp_ns": stamp, "sensor_values": {}}
            for topic, entries in data_by_topic.items():
                if TOPIC_TYPES[topic] == "image":
                    continue
                match = nearest(entries, stamp)
                if match is None:
                    item["sensor_values"][topic] = None
                    continue
                msg, offset_ms = match
                kind = TOPIC_TYPES[topic]
                values = imu_values(msg) if kind == "imu" else odom_values(msg)
                item["sensor_values"][topic] = {"time_offset_ms": offset_ms, **values}
            pair.append(item)
        outputs.append({"target_index": event["target_index"],
                        "source_interval_s": event["source_interval_s"],
                        "frames": pair})
    return outputs


def save_depth_overlays(events, images, support_by_target, output_dir):
    """追跡画素を深度画像へ重ねたframe pairを保存する。"""
    output_dir.mkdir(parents=True, exist_ok=True)
    for event in events:
        panels = []
        for suffix, stamp_key, pixel_key in (
                ("0", "source_stamp0_ns", "depth_pixel0"),
                ("1", "source_stamp1_ns", "depth_pixel1")):
            depth = images.get(event[stamp_key])
            if depth is None:
                continue
            # 約0.4～5 mを同じlinear grayscale範囲に固定し、点ごとの色変化を比較できるようにする。
            scaled = np.clip((depth - 0.4) / (5.0 - 0.4) * 255, 0, 255).astype(np.uint8)
            bgr = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
            bgr[depth <= 0] = (24, 24, 24)
            for match in event["matches"]:
                u, v = (int(round(value)) for value in match[pixel_key])
                cv2.circle(bgr, (u, v), 3, (255, 255, 255), 1, cv2.LINE_AA)
            # 実際のplane supportで選ばれたdepth画素は赤い十字で別表示する。
            for support in support_by_target.get(event["target_index"], []):
                if int(support["source_stamp_ns"]) != event[stamp_key]:
                    continue
                u, v = [int(round(value)) for value in support["depth_pixel"]]
                cv2.drawMarker(bgr, (u, v), (0, 0, 255), cv2.MARKER_CROSS,
                               17, 2, cv2.LINE_AA)
                cv2.putText(bgr, "mapper source {:.2f}m".format(
                    support["depth_m"]), (min(500, u + 8), max(16, v - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(bgr, "frame {} depth stamp {}".format(
                suffix, event[stamp_key]), (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, (255, 255, 255), 1, cv2.LINE_AA)
            panels.append(bgr)
        if len(panels) == 2:
            contact = np.concatenate(panels, axis=1)
            cv2.imwrite(str(output_dir / "black{:02d}_depth_track_overlay.png".format(
                event["target_index"])), contact)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("frame_events_csv", type=Path)
    parser.add_argument("track_summary", type=Path)
    parser.add_argument("map_support_summary", type=Path)
    parser.add_argument("forensic_dir", type=Path,
                        help="同一bag再生で保存したframe_pixels/frame_events CSVの場所")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--margin-s", type=float, default=0.75)
    args = parser.parse_args()
    events, stamps = read_tracks(args.track_summary)
    support_by_target = read_map_support_pixels(args.map_support_summary)
    data, images = extract_bag_data(args.bag, stamps, args.margin_s)
    if set(stamps) - set(images):
        missing = sorted(set(stamps) - set(images))
        raise RuntimeError("指定depth frameがbagにありません: " + repr(missing))
    poses = read_frame_pose(args.frame_events_csv)
    if set(stamps) - set(poses):
        raise RuntimeError("frame event poseが欠落するdepth stampがあります")
    patch_rows = depth_quality(events, images, support_by_target, poses)
    output = {
        "bag": str(args.bag), "target_indices": [5, 12, 13],
        "source_depth_frame_count": len(images),
        "sensor_context": sensor_context(events, stamps, data),
        "depth_patch_quality": patch_rows,
        # 実使用pixelが同stamp・同cellでどの地図セルへ投影され、
        # 既存ground値をどう更新したかをraw forensic CSVから直結する。
        "exact_support_fusion": read_exact_support_updates(args.forensic_dir, patch_rows),
        "limitations": [
            "raw /vio/odometry and /wheel/odometry are context signals, not independent ground truth",
            "depth patch spread can include real road slope/texture as well as sensor noise",
            "RGB feature tracking does not ensure the exact same depth surface at an occlusion edge",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2), encoding="utf-8")
    save_depth_overlays(events, images, support_by_target,
                        args.output.parent / "depth_track_overlays")
    for entry in output["depth_patch_quality"]:
        print("black", entry["target_index"])
        for frame in entry["frames"]:
            print(" frame", frame["frame"], "N", frame.get("feature_count"),
                  "3x3 spread", frame.get("p3x3_p10_p90_spread_m"),
                  "9x9 spread", frame.get("p9x9_p10_p90_spread_m"),
                  "valid3x3", frame.get("valid_fraction_3x3_p50"),
                  "mapper pixel", frame.get("map_support_pixel", {}).get("depth_pixel"))
    print("保存先:", args.output)


if __name__ == "__main__":
    main()
