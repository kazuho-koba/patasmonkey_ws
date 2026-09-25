#!/usr/bin/env python3
"""舗装面の画像特徴を追跡し、depthと撮像時TFによるodom再投影差を測る。

color/depth画像を再生成したりterrain nodeを動かしたりせず、保存済みblack地点supportの
source stampだけをMCAPから抜き出す。舗装面の下部ROIでShi-Tomasi特徴を検出し、
pyramidal Lucas-Kanadeを往復適用して同じ画像特徴の候補を得る。各RGB pixelに対応する
aligned depth pixelの3x3中央値を読み、frameごとに保存されたcamera->odom TFで3D化する。
対応点が少ない、depthに欠損がある、近傍depthが不連続な場合は統計から除外する。
"""

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


COLOR_TOPIC = "/oak/color/image_raw"
DEPTH_TOPIC = "/oak/depth/image_raw"
COLOR_K = (516.8594971, 574.2882690, 351.2757568, 215.2326202)
DEFAULT_DEPTH_K = (574.28826904, 574.28826904, 354.75085449, 215.23262024)


def read_csv(path):
    """UTF-8 CSVを辞書行として読み込む。対象は局所診断出力に限られる。"""
    with Path(path).open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def ros_stamp_ns(message):
    """ROS header stampを整数nanosecondへ変換する。"""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def image_to_bgr(message):
    """ROS Imageのrow paddingを除き、RGB/BGR colorをBGRへ揃える。"""
    encoding = message.encoding.lower()
    channels = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(encoding)
    if channels is None:
        raise ValueError("未対応color encoding: " + encoding)
    view = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )[:, : message.width * channels].reshape(message.height, message.width, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(view, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(view, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(view, cv2.COLOR_BGRA2BGR)
    return view.copy()


def depth_array(message):
    """16UC1/mono16 depthをrow padding込みで読み、mmからmへ変換する。"""
    if message.encoding not in ("16UC1", "mono16"):
        raise ValueError("未対応depth encoding: " + message.encoding)
    dtype = ">u2" if message.is_bigendian else "<u2"
    row_words = message.step // 2
    raw = np.frombuffer(message.data, dtype=dtype).reshape(
        message.height, row_words
    )[:, : message.width]
    return raw.astype(np.float32) * 0.001


def camera_transform(row):
    """forensic frame eventのcamera->odom quaternion/translationを行列化する。"""
    q = np.array([float(row[key]) for key in
                  ("camera_qx", "camera_qy", "camera_qz", "camera_qw")],
                 dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        raise ValueError("camera TF quaternionがゼロです")
    x, y, z, w = q / norm
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)
    translation = np.array([float(row[key]) for key in
                            ("camera_x_m", "camera_y_m", "camera_z_m")])
    return rotation, translation


def rgb_to_depth_pixel(u_rgb, v_rgb, depth_k):
    """EEPROM-derived RGB/depth K ratioでRGB preview pixelをdepth pixelへ戻す。"""
    fx_d, fy_d, cx_d, cy_d = depth_k
    fx_c, fy_c, cx_c, cy_c = COLOR_K
    return ((u_rgb - cx_c) * fx_d / fx_c + cx_d,
            (v_rgb - cy_c) * fy_d / fy_c + cy_d)


def sample_depth_patch(depth, u, v, spread_limit_m):
    """対応pixelの周囲3x3からvalid depth中央値と散らばりを返す。"""
    x, y = int(round(u)), int(round(v))
    if x < 1 or y < 1 or x >= depth.shape[1] - 1 or y >= depth.shape[0] - 1:
        return None
    patch = depth[y - 1:y + 2, x - 1:x + 2]
    valid = patch[np.isfinite(patch) & (patch >= 0.4) & (patch <= 5.0)]
    if valid.size < 5:
        return None
    median = float(np.median(valid))
    spread = float(np.percentile(valid, 90) - np.percentile(valid, 10))
    if spread > spread_limit_m:
        return None
    return median, spread


def depth_point_world(u_rgb, v_rgb, depth, pose, depth_k, spread_limit_m):
    """RGB特徴位置のdepthをpinhole back-projectし、保存済みTFでodomへ変換。"""
    u_d, v_d = rgb_to_depth_pixel(u_rgb, v_rgb, depth_k)
    sample = sample_depth_patch(depth, u_d, v_d, spread_limit_m)
    if sample is None:
        return None
    z_depth, spread = sample
    fx, fy, cx, cy = depth_k
    point_camera = np.array([(u_d - cx) * z_depth / fx,
                             (v_d - cy) * z_depth / fy, z_depth])
    rotation, translation = pose
    return rotation @ point_camera + translation, z_depth, spread, (u_d, v_d)


def road_mask(shape, top_fraction, side_fraction):
    """画像下部かつ左右端を避けたroad候補ROIを作る。semantic判定ではない。"""
    height, width = shape
    mask = np.zeros((height, width), dtype=np.uint8)
    top = int(round(height * top_fraction))
    left = int(round(width * side_fraction))
    right = width - left
    mask[top:, left:right] = 255
    return mask


def track_and_reproject(image0, image1, depth0, depth1, pose0, pose1,
                        depth_k, args):
    """2枚の舗装候補画像でKLT追跡し、3D world座標差を計算する。"""
    gray0 = cv2.cvtColor(image0, cv2.COLOR_BGR2GRAY)
    gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    mask = road_mask(gray0.shape, args.road_top_fraction, args.side_fraction)
    corners = cv2.goodFeaturesToTrack(
        gray0, maxCorners=args.max_features, qualityLevel=0.005,
        minDistance=7, blockSize=7, mask=mask,
    )
    if corners is None or len(corners) == 0:
        return [], 0, 0, 0, None

    lk = dict(winSize=(31, 31), maxLevel=4,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    forward, status_forward, _ = cv2.calcOpticalFlowPyrLK(gray0, gray1, corners, None, **lk)
    if forward is None:
        return [], len(corners), 0, 0, None
    backward, status_backward, _ = cv2.calcOpticalFlowPyrLK(gray1, gray0, forward, None, **lk)
    if backward is None:
        return [], len(corners), 0, 0, None
    p0 = corners.reshape(-1, 2)
    p1 = forward.reshape(-1, 2)
    p0_back = backward.reshape(-1, 2)
    fb_error = np.linalg.norm(p0 - p0_back, axis=1)
    mask1 = road_mask(gray1.shape, args.road_top_fraction, args.side_fraction)
    accepted = (status_forward.reshape(-1) > 0) & (status_backward.reshape(-1) > 0)
    accepted &= fb_error <= args.forward_backward_px
    accepted &= np.array([
        0 <= int(round(x)) < mask1.shape[1] and 0 <= int(round(y)) < mask1.shape[0]
        and mask1[int(round(y)), int(round(x))] > 0 for x, y in p1
    ])

    # 舗装面を局所平面とみなし、RANSAC homographyに合わないLK誤対応を落とす。
    # 平行な目地などを誤って追った点だけでは多数決できないよう、inlier数も下限を持つ。
    flow_indices = np.flatnonzero(accepted)
    if flow_indices.size < 4:
        return [], len(corners), int(flow_indices.size), 0, None
    homography, inlier_mask = cv2.findHomography(
        p0[flow_indices], p1[flow_indices], cv2.RANSAC,
        args.homography_reprojection_px,
    )
    if homography is None or inlier_mask is None:
        return [], len(corners), int(flow_indices.size), 0, None
    inlier_indices = flow_indices[inlier_mask.reshape(-1) > 0]
    if inlier_indices.size < args.homography_min_inliers:
        return [], len(corners), int(flow_indices.size), int(inlier_indices.size), homography.tolist()

    matches = []
    for feature_id in inlier_indices:
        first = depth_point_world(*p0[feature_id], depth0, pose0, depth_k,
                                  args.depth_patch_spread_m)
        second = depth_point_world(*p1[feature_id], depth1, pose1, depth_k,
                                   args.depth_patch_spread_m)
        if first is None or second is None:
            continue
        world0, z0, spread0, depth_pixel0 = first
        world1, z1, spread1, depth_pixel1 = second
        delta = world1 - world0
        # world差を「TF並進」「同じrayをpose0/pose1で回した差」「depth/ray差」に分ける。
        # これは記録された入力の代数分解で、どの項が誤差原因かを単独で証明するものではない。
        fx, fy, cx, cy = depth_k
        point_camera0 = np.array([
            (depth_pixel0[0] - cx) * z0 / fx,
            (depth_pixel0[1] - cy) * z0 / fy,
            z0,
        ])
        point_camera1 = np.array([
            (depth_pixel1[0] - cx) * z1 / fx,
            (depth_pixel1[1] - cy) * z1 / fy,
            z1,
        ])
        rotation0, translation0 = pose0
        rotation1, translation1 = pose1
        translation_term = translation1 - translation0
        rotation_term = (rotation1 @ point_camera0) - (rotation0 @ point_camera0)
        ray_term = rotation1 @ (point_camera1 - point_camera0)
        matches.append({
            "feature_id": int(feature_id),
            "rgb0": p0[feature_id].tolist(), "rgb1": p1[feature_id].tolist(),
            "depth_pixel0": list(depth_pixel0), "depth_pixel1": list(depth_pixel1),
            "depth_m0": z0, "depth_m1": z1,
            "depth_patch_spread_m0": spread0, "depth_patch_spread_m1": spread1,
            "forward_backward_error_px": float(fb_error[feature_id]),
            "odom_xyz0_m": world0.tolist(), "odom_xyz1_m": world1.tolist(),
            "delta_xyz_m": delta.tolist(),
            "camera_translation_term_xyz_m": translation_term.tolist(),
            "camera_rotation_term_xyz_m": rotation_term.tolist(),
            "ray_and_depth_term_xyz_m": ray_term.tolist(),
            "delta_xy_m": float(np.linalg.norm(delta[:2])),
            "delta_z_m": float(delta[2]),
            "delta_3d_m": float(np.linalg.norm(delta)),
        })
    return (matches, len(corners), int(flow_indices.size),
            int(inlier_indices.size), homography.tolist())


def load_frame_messages(bag_dir, requested_stamps, rgb_tolerance_ms):
    """対象depth exact stampと、それに近いRGB画像だけをMCAPから抽出する。"""
    requested = sorted(set(requested_stamps))
    rgb_best, depth_exact = {}, {}
    tolerance_ns = int(rgb_tolerance_ms * 1e6)
    rclpy.init()
    try:
        for bagfile in sorted(Path(bag_dir).glob("*.mcap")):
            with bagfile.open("rb") as stream:
                for _, channel, record in make_reader(stream).iter_messages(
                        topics=[COLOR_TOPIC, DEPTH_TOPIC]):
                    message = deserialize_message(record.data, Image)
                    stamp = ros_stamp_ns(message)
                    if channel.topic == DEPTH_TOPIC and stamp in requested:
                        depth_exact[stamp] = (message.width, message.height,
                                              message.header.frame_id,
                                              depth_array(message))
                    elif channel.topic == COLOR_TOPIC:
                        index = int(np.searchsorted(requested, stamp))
                        candidates = [i for i in (index - 1, index)
                                      if 0 <= i < len(requested)]
                        if not candidates:
                            continue
                        target = min((requested[i] for i in candidates),
                                     key=lambda value: abs(value - stamp))
                        delta = abs(stamp - target)
                        if delta <= tolerance_ns and (
                                target not in rgb_best or delta < rgb_best[target][0]):
                            rgb_best[target] = (delta, stamp, message.width, message.height,
                                                message.header.frame_id, image_to_bgr(message))
    finally:
        rclpy.shutdown()
    return rgb_best, depth_exact


def create_event_pairs(summary):
    """同じmap black地点の異なるsource stamp2点を時系列pairへ並べる。"""
    pairs = []
    for target in summary["targets"]:
        images = target.get("images", [])
        by_stamp = {int(item["source_stamp_ns"]): item for item in images}
        if len(by_stamp) < 2:
            continue
        ordered = sorted(by_stamp.values(), key=lambda item: item["source_stamp_ns"])
        pairs.append({"target": target, "first": ordered[0], "second": ordered[-1]})
    return pairs


def summarize_matches(matches):
    """再投影差のmedian/p90と閾値内割合を計算する。"""
    if not matches:
        return {"valid_matches": 0}
    values = {name: np.array([row[name] for row in matches], dtype=np.float64)
              for name in ("delta_xy_m", "delta_z_m", "delta_3d_m",
                           "forward_backward_error_px")}
    decomposition = {}
    for key, label in (("camera_translation_term_xyz_m", "camera_translation"),
                       ("camera_rotation_term_xyz_m", "camera_rotation"),
                       ("ray_and_depth_term_xyz_m", "ray_and_depth")):
        vectors = np.array([row[key] for row in matches], dtype=np.float64)
        decomposition[label + "_median_xyz_m"] = np.median(vectors, axis=0).tolist()
    return {
        "valid_matches": len(matches),
        "delta_xy_m_median": float(np.median(values["delta_xy_m"])),
        "delta_xy_m_p90": float(np.percentile(values["delta_xy_m"], 90)),
        "abs_delta_z_m_median": float(np.median(np.abs(values["delta_z_m"]))),
        "abs_delta_z_m_p90": float(np.percentile(np.abs(values["delta_z_m"]), 90)),
        "delta_3d_m_median": float(np.median(values["delta_3d_m"])),
        "delta_3d_m_p90": float(np.percentile(values["delta_3d_m"], 90)),
        "matches_with_abs_delta_z_under_0p05m": int(np.count_nonzero(
            np.abs(values["delta_z_m"]) <= 0.05)),
        "matches_with_delta_3d_under_0p10m": int(np.count_nonzero(
            values["delta_3d_m"] <= 0.10)),
        "forward_backward_px_median": float(np.median(values["forward_backward_error_px"])),
        "algebraic_decomposition": decomposition,
    }


def draw_matches(image0, image1, matches, title):
    """RGB追跡結果をside-by-sideで描画し、特徴対応を目視確認可能にする。"""
    height = max(image0.shape[0], image1.shape[0])
    width = image0.shape[1]
    canvas = np.zeros((height, width * 2, 3), dtype=np.uint8)
    canvas[:image0.shape[0], :width] = image0
    canvas[:image1.shape[0], width:width * 2] = image1
    cv2.putText(canvas, title, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (255, 255, 255), 1, cv2.LINE_AA)
    for index, match in enumerate(matches[:150]):
        color = (40 + (index * 67) % 215, 50 + (index * 41) % 205,
                 60 + (index * 97) % 195)
        p0 = tuple(np.rint(match["rgb0"]).astype(int))
        p1 = tuple(np.rint(match["rgb1"]).astype(int) + np.array([width, 0]))
        cv2.circle(canvas, p0, 3, color, -1, cv2.LINE_AA)
        cv2.circle(canvas, p1, 3, color, -1, cv2.LINE_AA)
        cv2.line(canvas, p0, p1, color, 1, cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("color_projection_json", type=Path)
    parser.add_argument("pose_events_csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-targets", type=int, default=100)
    parser.add_argument("--max-color-depth-offset-ms", type=float, default=100.0)
    parser.add_argument("--road-top-fraction", type=float, default=0.60)
    parser.add_argument("--side-fraction", type=float, default=0.20)
    parser.add_argument("--max-features", type=int, default=500)
    parser.add_argument("--forward-backward-px", type=float, default=1.0)
    parser.add_argument("--homography-reprojection-px", type=float, default=2.0)
    parser.add_argument("--homography-min-inliers", type=int, default=8)
    parser.add_argument("--depth-patch-spread-m", type=float, default=0.15)
    parser.add_argument("--depth-k", type=float, nargs=4, default=DEFAULT_DEPTH_K,
                        metavar=("FX", "FY", "CX", "CY"))
    args = parser.parse_args()

    projection = json.loads(args.color_projection_json.read_text(encoding="utf-8"))
    pairs = create_event_pairs(projection)[:args.max_targets]
    stamps = sorted({int(item[key]["source_stamp_ns"])
                     for item in pairs for key in ("first", "second")})
    rgb, depth = load_frame_messages(args.bag, stamps, args.max_color_depth_offset_ms)
    pose_by_stamp = {}
    for row in read_csv(args.pose_events_csv):
        stamp = int(row["source_stamp_ns"])
        if stamp in stamps and stamp not in pose_by_stamp:
            pose_by_stamp[stamp] = camera_transform(row)

    args.output.mkdir(parents=True, exist_ok=True)
    event_results, contact_tiles = [], []
    for pair_index, pair in enumerate(pairs, start=1):
        target = pair["target"]
        first, second = pair["first"], pair["second"]
        stamp0, stamp1 = int(first["source_stamp_ns"]), int(second["source_stamp_ns"])
        entry = {"target_index": target["index"], "max_cause": target["max_cause"],
                 "map_stamp_ns": target["map_stamp_ns"],
                 "source_stamp0_ns": stamp0, "source_stamp1_ns": stamp1,
                 "source_interval_s": (stamp1 - stamp0) / 1e9,
                 "support_cell0": first["selected_support_cell"],
                 "support_cell1": second["selected_support_cell"],
                 "support_map_elevation0_m": first["support_map_elevation_m"],
                 "support_map_elevation1_m": second["support_map_elevation_m"],
                 "color_frame0_found": stamp0 in rgb, "color_frame1_found": stamp1 in rgb,
                 "depth_frame0_found": stamp0 in depth, "depth_frame1_found": stamp1 in depth,
                 "pose0_found": stamp0 in pose_by_stamp, "pose1_found": stamp1 in pose_by_stamp}
        if not all(entry[key] for key in ("color_frame0_found", "color_frame1_found",
                                          "depth_frame0_found", "depth_frame1_found",
                                          "pose0_found", "pose1_found")):
            event_results.append(entry)
            continue
        rgb0 = rgb[stamp0][5]
        rgb1 = rgb[stamp1][5]
        depth0 = depth[stamp0][3]
        depth1 = depth[stamp1][3]
        matches, detected, flow_count, homography_count, homography = track_and_reproject(
            rgb0, rgb1, depth0, depth1, pose_by_stamp[stamp0], pose_by_stamp[stamp1],
            tuple(args.depth_k), args)
        entry.update({"detected_features": detected, "flow_valid_in_road_roi": flow_count,
                      "road_plane_homography_inliers": homography_count,
                      "road_plane_homography": homography,
                      "statistics": summarize_matches(matches), "matches": matches,
                      "rgb_stamp_offset0_ms": (rgb[stamp0][1] - stamp0) / 1e6,
                      "rgb_stamp_offset1_ms": (rgb[stamp1][1] - stamp1) / 1e6,
                      "image_width": rgb[stamp0][2], "image_height": rgb[stamp0][3],
                      "depth_frame_id": depth[stamp0][2], "rgb_frame_id": rgb[stamp0][4]})
        output_image = draw_matches(rgb0, rgb1, matches,
                                    "black {} cause={} dt={:.3f}s tracked={}/{} depth={}".format(
                                        target["index"], target["max_cause"],
                                        entry["source_interval_s"], len(matches), detected,
                                        entry["depth_frame_id"]))
        image_name = "black{:02d}_tracked_features.png".format(target["index"])
        cv2.imwrite(str(args.output / image_name), output_image)
        entry["match_image"] = image_name
        contact_tiles.append(cv2.resize(output_image, (960, 300)))
        event_results.append(entry)

    if contact_tiles:
        cv2.imwrite(str(args.output / "road_feature_tracks_contact_sheet.png"),
                    np.vstack(contact_tiles))
    valid_events = [row for row in event_results
                    if row.get("statistics", {}).get("valid_matches", 0) > 0]
    all_matches = [match for row in valid_events for match in row["matches"]]
    summary = {
        "bag": str(args.bag), "event_pair_count": len(pairs),
        "event_pairs_with_all_inputs": sum(
            all(row.get(key, False) for key in (
                "color_frame0_found", "color_frame1_found", "depth_frame0_found",
                "depth_frame1_found", "pose0_found", "pose1_found"))
            for row in event_results),
        "event_pairs_with_valid_depth_tracks": len(valid_events),
        "total_valid_feature_matches": len(all_matches),
        "aggregate_statistics": summarize_matches(all_matches),
        "parameters": {"road_top_fraction": args.road_top_fraction,
                       "side_fraction": args.side_fraction,
                       "max_features": args.max_features,
                       "forward_backward_px": args.forward_backward_px,
                       "homography_reprojection_px": args.homography_reprojection_px,
                       "homography_min_inliers": args.homography_min_inliers,
                       "depth_patch_spread_m": args.depth_patch_spread_m,
                       "depth_k": list(args.depth_k), "color_k": list(COLOR_K)},
        "events": event_results,
        "limitations": [
            "color/depth header stampは異なり、frame毎に最大指定許容差が残る",
            "KLT追跡とroad ROIは同一物理点を保証せず、各eventの対応図で目視確認が必要",
            "同じ特徴のworld差にはdepth誤差、camera calibration誤差、TF誤差が混在する",
            "camera poseはdepth撮像時刻のTFで、RGB撮像時刻へ補間していない",
        ],
    }
    (args.output / "road_feature_track_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("対象pair:", len(pairs), "必要データあり:", summary["event_pairs_with_all_inputs"],
          "depth追跡有効:", len(valid_events),
          "追跡+depth有効点:", len(all_matches))
    print("出力:", args.output)


if __name__ == "__main__":
    main()
