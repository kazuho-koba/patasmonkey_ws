#!/usr/bin/env python3
"""mapper実使用depth support位置をRGB frame間で直接追跡する。

以前の任意Shi-Tomasi特徴群を比較するのではなく、terrain forensicで選ばれたdepth
support pixelをRGB preview座標へ写し、その位置を近接するカラー画像間で追跡する。
forward/backward Lucas-Kanadeと道路ROI homographyの両方で追跡し、depth/Timestamped TF
でodomへ戻した時の同一舗装特徴残差、および次frameのmapper-selected supportとの間隔を
比較する。処理対象は黒地点5/12/13のsource frame各2枚だけ。

7月bagにはCameraInfoがないため、既存診断と同じEEPROM Kを使う。カラー画像とdepthの
header時刻には差があり、TFはdepth時刻のものしかCSVにない。この解析はmarker近傍の
対応診断であって、独立ground truthや厳密な色-depth同期検証ではない。
"""

import argparse
import bisect
import csv
import json
from pathlib import Path

import cv2
import numpy as np


COLOR_TOPIC = "/oak/color/image_raw"
DEPTH_TOPIC = "/oak/depth/image_raw"
COLOR_K = (516.8594971, 574.2882690, 351.2757568, 215.2326202)
DEPTH_K = (574.28826904, 574.28826904, 354.75085449, 215.23262024)


def stamp_ns(message):
    """ROS header stampをinteger nanosecondsにする。"""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def color_bgr(message):
    """row paddingを除いて、サポート対象のcolor encodingをBGRへ揃える。"""
    channels = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(
        message.encoding.lower())
    if channels is None:
        raise ValueError("未対応color encoding: " + message.encoding)
    view = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step)[:, :message.width * channels].reshape(
            message.height, message.width, channels)
    if message.encoding.lower() == "rgb8":
        return cv2.cvtColor(view, cv2.COLOR_RGB2BGR)
    if message.encoding.lower() == "rgba8":
        return cv2.cvtColor(view, cv2.COLOR_RGBA2BGR)
    if message.encoding.lower() == "bgra8":
        return cv2.cvtColor(view, cv2.COLOR_BGRA2BGR)
    return view.copy()


def depth_meters(message):
    """16UC1/mono16 depthをrow stepに従って読み、mmからmへ直す。"""
    if message.encoding not in ("16UC1", "mono16"):
        raise ValueError("未対応depth encoding: " + message.encoding)
    dtype = ">u2" if message.is_bigendian else "<u2"
    raw = np.frombuffer(message.data, dtype=dtype).reshape(
        message.height, message.step // 2)[:, :message.width]
    return raw.astype(np.float32) * 0.001


def load_frames(bag_dir, requested_stamps, color_tolerance_ms):
    """対象depth frameと、それぞれへ最も近いRGB画像だけをMCAPから取る。"""
    import rclpy
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    stamps = sorted(set(requested_stamps))
    rgb_best, depth = {}, {}
    tolerance_ns = int(color_tolerance_ms * 1e6)
    rclpy.init()
    try:
        for path in sorted(Path(bag_dir).glob("*.mcap")):
            with path.open("rb") as stream:
                for _, channel, record in make_reader(stream).iter_messages(
                        topics=[COLOR_TOPIC, DEPTH_TOPIC]):
                    topic = channel.topic if hasattr(channel, "topic") else channel
                    msg = deserialize_message(record.data, Image)
                    stamp = stamp_ns(msg)
                    if topic == DEPTH_TOPIC:
                        if stamp in stamps:
                            depth[stamp] = (depth_meters(msg), msg.header.frame_id)
                        continue
                    index = bisect.bisect_left(stamps, stamp)
                    candidates = [i for i in (index - 1, index)
                                  if 0 <= i < len(stamps)]
                    if not candidates:
                        continue
                    target = min((stamps[i] for i in candidates),
                                 key=lambda value: abs(value - stamp))
                    delta = abs(stamp - target)
                    if delta > tolerance_ns:
                        continue
                    prior = rgb_best.get(target)
                    if prior is None or delta < prior[0]:
                        rgb_best[target] = (
                            delta, stamp, color_bgr(msg), msg.header.frame_id)
    finally:
        rclpy.shutdown()
    return rgb_best, depth


def read_csv(path):
    """CSVを小さな対象表として読み込む。"""
    with Path(path).open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def depth_to_color(pixel):
    """共通optical rayをdepth Kからcolor preview Kへ写す（既存診断と同じ式）。"""
    u, v = pixel
    fx_d, fy_d, cx_d, cy_d = DEPTH_K
    fx_c, fy_c, cx_c, cy_c = COLOR_K
    return np.asarray([fx_c * (u - cx_d) / fx_d + cx_c,
                       fy_c * (v - cy_d) / fy_d + cy_c], dtype=np.float32)


def rgb_to_depth(pixel):
    """color preview座標をdepth output座標へ戻す。"""
    u, v = pixel
    fx_d, fy_d, cx_d, cy_d = DEPTH_K
    fx_c, fy_c, cx_c, cy_c = COLOR_K
    return np.asarray([fx_d * (u - cx_c) / fx_c + cx_d,
                       fy_d * (v - cy_c) / fy_c + cy_d], dtype=np.float64)


def pose_matrix(row):
    """frame eventのcamera→odom quaternionとtranslationを行列化する。"""
    q = np.asarray([float(row["camera_qx"]), float(row["camera_qy"]),
                    float(row["camera_qz"]), float(row["camera_qw"])])
    x, y, z, w = q / np.linalg.norm(q)
    rotation = np.asarray([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)
    translation = np.asarray([float(row["camera_x_m"]),
                              float(row["camera_y_m"]),
                              float(row["camera_z_m"])])
    return rotation, translation


def sample_depth(depth, rgb_pixel):
    """対応RGB位置をdepthへ戻し、3x3のvalid depth median/spreadを計算する。"""
    u, v = np.rint(rgb_to_depth(rgb_pixel)).astype(int)
    if u < 1 or v < 1 or u >= depth.shape[1]-1 or v >= depth.shape[0]-1:
        return None
    patch = depth[v-1:v+2, u-1:u+2]
    valid = patch[(patch >= 0.4) & (patch <= 5.0)]
    if valid.size < 5:
        return None
    return {"pixel_uv": [int(u), int(v)], "depth_m": float(np.median(valid)),
            "valid_count": int(valid.size),
            "p10_p90_spread_m": float(np.percentile(valid, 90)
                                      - np.percentile(valid, 10))}


def world_from_rgb_point(rgb_pixel, depth, pose):
    """RGB pixelに対応するdepth patchをback-projectし、odom xyzへ変換する。"""
    sample = sample_depth(depth, rgb_pixel)
    if sample is None:
        return None
    u, v = sample["pixel_uv"]
    z_depth = sample["depth_m"]
    fx, fy, cx, cy = DEPTH_K
    camera = np.asarray([(u-cx)*z_depth/fx, (v-cy)*z_depth/fy, z_depth])
    rotation, translation = pose
    return {**sample, "world_xyz_m": (rotation @ camera + translation).tolist()}


def local_road_homography(gray0, gray1, seed):
    """seed周辺の路面cornerを追い、局所road homographyと追跡点数を求める。"""
    h, w = gray0.shape
    mask = np.zeros((h, w), dtype=np.uint8)
    # mapper supportは道路内に表示される。中心近傍±150 pxのみに検出を絞る。
    x, y = np.rint(seed).astype(int)
    x0, x1 = max(0, x-150), min(w, x+151)
    y0, y1 = max(int(h*0.45), y-100), min(h, y+101)
    cv2.rectangle(mask, (x0, y0), (x1-1, y1-1), 255, -1)
    points0 = cv2.goodFeaturesToTrack(gray0, maxCorners=300,
                                      qualityLevel=0.005, minDistance=6,
                                      blockSize=7, mask=mask)
    if points0 is None or len(points0) < 8:
        return None, {"detected_local_features": 0 if points0 is None else len(points0),
                      "tracked_forward_backward": 0, "homography_inliers": 0}
    lk = dict(winSize=(31, 31), maxLevel=4,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    points1, status1, _ = cv2.calcOpticalFlowPyrLK(gray0, gray1, points0, None, **lk)
    if points1 is None:
        return None, {"detected_local_features": len(points0),
                      "tracked_forward_backward": 0, "homography_inliers": 0}
    back, status_back, _ = cv2.calcOpticalFlowPyrLK(gray1, gray0, points1, None, **lk)
    if back is None:
        return None, {"detected_local_features": len(points0),
                      "tracked_forward_backward": 0, "homography_inliers": 0}
    p0 = points0.reshape(-1, 2)
    p1 = points1.reshape(-1, 2)
    fb = np.linalg.norm(back.reshape(-1, 2) - p0, axis=1)
    good = (status1.reshape(-1) > 0) & (status_back.reshape(-1) > 0) & (fb <= 1.0)
    if np.count_nonzero(good) < 8:
        return None, {"detected_local_features": len(points0),
                      "tracked_forward_backward": int(np.count_nonzero(good)),
                      "homography_inliers": 0}
    homography, inlier_mask = cv2.findHomography(p0[good], p1[good], cv2.RANSAC, 2.0)
    if homography is None or inlier_mask is None:
        return None, {"detected_local_features": len(points0),
                      "tracked_forward_backward": int(np.count_nonzero(good)),
                      "homography_inliers": 0}
    inliers = int(np.count_nonzero(inlier_mask))
    return homography, {"detected_local_features": int(len(points0)),
                        "tracked_forward_backward": int(np.count_nonzero(good)),
                        "homography_inliers": inliers,
                        "forward_backward_error_median_px": float(np.median(fb[good]))}


def track_seed(image0, image1, seed):
    """mapper markerそのものをLKでforward/backward追跡する。"""
    gray0, gray1 = cv2.cvtColor(image0, cv2.COLOR_BGR2GRAY), cv2.cvtColor(
        image1, cv2.COLOR_BGR2GRAY)
    point0 = np.asarray(seed, dtype=np.float32).reshape(1, 1, 2)
    lk = dict(winSize=(31, 31), maxLevel=4,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.005))
    point1, status1, error1 = cv2.calcOpticalFlowPyrLK(
        gray0, gray1, point0, None, **lk)
    if point1 is None or not status1[0, 0]:
        return None, {"lk_success": False}
    back, status_back, _ = cv2.calcOpticalFlowPyrLK(
        gray1, gray0, point1, None, **lk)
    if back is None or not status_back[0, 0]:
        return point1.reshape(2).astype(float), {"lk_success": False}
    endpoint = point1.reshape(2).astype(float)
    fb = float(np.linalg.norm(back.reshape(2) - np.asarray(seed)))
    return endpoint, {"lk_success": True, "forward_backward_error_px": fb,
                      "lk_error": float(error1[0, 0])}


def project_homography(homography, point):
    """2D homographyで1点を別frameへ写す。"""
    p = np.asarray([[[point[0], point[1]]]], dtype=np.float32)
    return cv2.perspectiveTransform(p, homography).reshape(2).astype(float)


def draw_pair(image0, image1, marker0, marker1, lk_point, h_point,
              target_index, stats, output_path):
    """source/target mapper markerと2種の追跡先をside-by-sideで保存する。"""
    width = image0.shape[1]
    canvas = np.concatenate((image0.copy(), image1.copy()), axis=1)
    offset = np.asarray([width, 0])
    def cross(point, color, label):
        if point is None:
            return
        location = tuple(np.rint(point).astype(int))
        cv2.drawMarker(canvas, location, color, cv2.MARKER_CROSS, 17, 2, cv2.LINE_AA)
        cv2.putText(canvas, label, (location[0]+7, max(18, location[1]-6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1, cv2.LINE_AA)
    cross(marker0, (255, 255, 255), "source mapper pixel")
    cross(marker1 + offset, (255, 255, 255), "next mapper pixel")
    cross(lk_point + offset if lk_point is not None else None,
          (40, 220, 255), "LK source track")
    cross(h_point + offset if h_point is not None else None,
          (255, 120, 30), "road H source track")
    cv2.line(canvas, tuple(np.rint(marker0).astype(int)),
             tuple(np.rint((h_point if h_point is not None else marker1)+offset).astype(int)),
             (255, 120, 30), 1, cv2.LINE_AA)
    title = "black {} mapper-seeded RGB tracking".format(target_index)
    cv2.putText(canvas, title, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, json.dumps(stats, ensure_ascii=False)[:160], (8, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(output_path), canvas)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("color_projection_summary", type=Path)
    parser.add_argument("frame_events_csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--color-tolerance-ms", type=float, default=60.0)
    args = parser.parse_args()

    summary = json.loads(args.color_projection_summary.read_text(encoding="utf-8"))
    # 対応表に含まれる全地点を使う。代表例だけに絞ると、低品質追跡などの
    # 個別例に結果が引っ張られるため、同一条件でまとめて分布を確認する。
    targets = summary["targets"]
    requested = sorted({int(image["source_stamp_ns"])
                        for target in targets for image in target.get("images", [])})
    rgb, depth = load_frames(args.bag, requested, args.color_tolerance_ms)
    pose_by_stamp = {}
    for row in read_csv(args.frame_events_csv):
        stamp = int(row["source_stamp_ns"])
        if stamp in requested and stamp not in pose_by_stamp:
            pose_by_stamp[stamp] = pose_matrix(row)

    args.output.mkdir(parents=True, exist_ok=True)
    output_targets = []
    for target in targets:
        images = sorted(target["images"], key=lambda item: int(item["source_stamp_ns"]))
        if len(images) != 2:
            continue
        first, second = images
        stamp0, stamp1 = int(first["source_stamp_ns"]), int(second["source_stamp_ns"])
        result = {"target_index": int(target["index"]),
                  "hazard_cell": [int(target["odom_cell_x"]), int(target["odom_cell_y"])],
                  "source_interval_s": (stamp1-stamp0)/1e9,
                  "rgb_offset0_ms": (rgb[stamp0][1]-stamp0)/1e6 if stamp0 in rgb else None,
                  "rgb_offset1_ms": (rgb[stamp1][1]-stamp1)/1e6 if stamp1 in rgb else None,
                  "support_cells": [first["selected_support_cell"],
                                    second["selected_support_cell"]],
                  "depth_pixels": [first["depth_pixel"], second["depth_pixel"]],
                  "depth_m": [first["depth_m"], second["depth_m"]]}
        needed = (stamp0 in rgb and stamp1 in rgb and stamp0 in depth and stamp1 in depth
                  and stamp0 in pose_by_stamp and stamp1 in pose_by_stamp)
        result["all_inputs_found"] = bool(needed)
        if not needed:
            output_targets.append(result)
            continue

        image0, image1 = rgb[stamp0][2], rgb[stamp1][2]
        marker0 = depth_to_color(first["depth_pixel"])
        marker1 = depth_to_color(second["depth_pixel"])
        gray0, gray1 = cv2.cvtColor(image0, cv2.COLOR_BGR2GRAY), cv2.cvtColor(
            image1, cv2.COLOR_BGR2GRAY)
        lk_point, lk_stats = track_seed(image0, image1, marker0)
        homography, h_stats = local_road_homography(gray0, gray1, marker0)
        h_point = project_homography(homography, marker0) if homography is not None else None
        result.update({"source_marker_rgb_uv": marker0.tolist(),
                       "next_marker_rgb_uv": marker1.tolist(),
                       "seed_lk": lk_stats, "local_road_homography": h_stats,
                       "lk_endpoint_rgb_uv": lk_point.tolist() if lk_point is not None else None,
                       "homography_endpoint_rgb_uv": h_point.tolist() if h_point is not None else None,
                       "homography_to_target_marker_px": (
                           float(np.linalg.norm(h_point-marker1)) if h_point is not None else None),
                       "lk_to_target_marker_px": (
                           float(np.linalg.norm(lk_point-marker1)) if lk_point is not None else None),
                       "lk_to_homography_endpoint_px": (
                           float(np.linalg.norm(lk_point-h_point))
                           if lk_point is not None and h_point is not None else None)})

        source_world = world_from_rgb_point(marker0, depth[stamp0][0], pose_by_stamp[stamp0])
        next_support_world = world_from_rgb_point(marker1, depth[stamp1][0], pose_by_stamp[stamp1])
        result["source_support_rgb_sample_world"] = source_world
        result["next_support_rgb_sample_world"] = next_support_world
        if lk_point is not None:
            tracked_world = world_from_rgb_point(lk_point, depth[stamp1][0], pose_by_stamp[stamp1])
            result["lk_tracked_source_world_in_next_frame"] = tracked_world
            if source_world and tracked_world:
                delta = np.asarray(tracked_world["world_xyz_m"]) - np.asarray(
                    source_world["world_xyz_m"])
                result["lk_source_feature_world_delta_xyz_m"] = delta.tolist()
                result["lk_source_feature_world_delta_xy_m"] = float(np.linalg.norm(delta[:2]))
                result["lk_source_feature_world_delta_z_m"] = float(delta[2])
        if h_point is not None:
            tracked_world = world_from_rgb_point(h_point, depth[stamp1][0], pose_by_stamp[stamp1])
            result["homography_tracked_source_world_in_next_frame"] = tracked_world
            if source_world and tracked_world:
                delta = np.asarray(tracked_world["world_xyz_m"]) - np.asarray(
                    source_world["world_xyz_m"])
                result["homography_source_feature_world_delta_xyz_m"] = delta.tolist()
                result["homography_source_feature_world_delta_xy_m"] = float(
                    np.linalg.norm(delta[:2]))
                result["homography_source_feature_world_delta_z_m"] = float(delta[2])
        filename = "black{:02d}_mapper_seeded_rgb_track.png".format(target["index"])
        draw_pair(image0, image1, marker0, marker1, lk_point, h_point,
                  target["index"], {"LK_fb": lk_stats.get("forward_backward_error_px"),
                                   "H_inliers": h_stats["homography_inliers"],
                                   "H_to_mapper_px": result["homography_to_target_marker_px"]},
                  args.output / filename)
        result["overlay"] = filename
        output_targets.append(result)

    payload = {
        "bag": str(args.bag), "color_k": COLOR_K, "depth_k": DEPTH_K,
        "pixel_mapping": "depth pixel to color via K ratio; reverse for depth sample",
        "targets": output_targets,
        "limitations": [
            "support marker pixel is mapped from RGB-depth K ratio; July bag lacks CameraInfo",
            "nearest RGB stamp differs from depth stamp; odom transform is depth-time pose",
            "LK tracks local appearance and homography assumes a locally planar road; neither is ground truth",
            "the target support pixel in the next frame is a different selected cell, not necessarily the same world point",
            "the depth patch at tracked color location can include a nearby different surface",
        ],
    }
    out_json = args.output / "mapper_seeded_rgb_tracks.json"
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print("target pairs:", len(output_targets), "inputs present:",
          sum(row["all_inputs_found"] for row in output_targets))
    for row in output_targets:
        print("black", row["target_index"], "LK", row.get("seed_lk"),
              "H", row.get("local_road_homography"),
              "H-marker px", row.get("homography_to_target_marker_px"),
              "world dz", row.get("homography_source_feature_world_delta_z_m"))
    print("saved:", out_json)


if __name__ == "__main__":
    main()
