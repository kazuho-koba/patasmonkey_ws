#!/usr/bin/env python3
"""ground融合前後のdepth候補をbag画像に重ね、入力pixelを目視できる形で保存する。

Diagnostic CSVのfusion mode 2（低い候補によるground置換）を抽出し、旧・新候補の
撮像時刻に近いRGB/depth frameとcamera_infoをMCAPから読む。source pixelはdepth画像
上に確実に描画し、RGB画像は寸法が一致する場合だけ同じpixelを参考表示する。
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
from sensor_msgs.msg import CameraInfo, Image, Imu


IMAGE_TYPES = {
    "/oak/color/image_raw": ("color", Image),
    "/oak/depth/image_raw": ("depth", Image),
}
INFO_TOPICS = {"/oak/depth/camera_info", "/oak/color/camera_info"}
IMU_TOPIC = "/wit/imu"


def stamp_ns(message):
    """ROS header stampを整数nsへ変換する。"""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def rpy_degrees(q):
    """WIT quaternionを比較用のroll/pitch/yawへ変換する。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [float(np.degrees(value)) for value in (roll, pitch, yaw)]


def image_to_array(message, kind):
    """Imageのrow padding/endianを尊重し、OpenCV表示用配列を返す。"""
    if kind == "color":
        encoding = message.encoding.lower()
        channels = 3 if encoding in ("rgb8", "bgr8") else 4 if encoding == "rgba8" else 0
        if channels == 0:
            raise ValueError("未対応color encoding: " + encoding)
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )[:, :message.width * channels]
        image = rows.reshape(message.height, message.width, channels)
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        return image.copy()
    dtype = ">u2" if message.is_bigendian else "<u2"
    words_per_row = message.step // 2
    return np.frombuffer(message.data, dtype=dtype).reshape(
        message.height, words_per_row
    )[:, :message.width].copy()


def depth_display(depth):
    """距離比較用に、全画像共通の0.4–6 m色スケールを適用する。"""
    metres = depth.astype(np.float32) * 0.001
    valid = np.isfinite(metres) & (metres >= 0.4) & (metres <= 6.0)
    gray = np.zeros(depth.shape, dtype=np.uint8)
    gray[valid] = np.rint((metres[valid] - 0.4) * (255.0 / 5.6)).astype(np.uint8)
    result = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    result[~valid] = 0
    return result


def depth_patch_statistics(depth, pixel_u, pixel_v, radius=16, stride=4):
    """source pixel周辺を実際のsampling間隔で調べ、深度の孤立外れ値を判別する。"""
    y0, y1 = max(0, pixel_v - radius), min(depth.shape[0], pixel_v + radius + 1)
    x0, x1 = max(0, pixel_u - radius), min(depth.shape[1], pixel_u + radius + 1)
    patch = depth[y0:y1:stride, x0:x1:stride].astype(np.float32) * 0.001
    valid = patch[np.isfinite(patch) & (patch >= 0.4) & (patch <= 6.0)]
    center = float(depth[pixel_v, pixel_u]) * 0.001
    if valid.size == 0:
        return {"pixel_depth_m": center, "valid_count": 0,
                "sampled_count": int(patch.size)}
    return {
        "pixel_depth_m": center,
        "valid_count": int(valid.size),
        "sampled_count": int(patch.size),
        "valid_fraction": float(valid.size / patch.size),
        "min_m": float(valid.min()),
        "p10_m": float(np.percentile(valid, 10)),
        "median_m": float(np.median(valid)),
        "p90_m": float(np.percentile(valid, 90)),
        "max_m": float(valid.max()),
        "std_m": float(valid.std()),
        "window_px": [int(x0), int(y0), int(x1 - x0), int(y1 - y0)],
        "sampling_stride_px": stride,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="MCAP rosbag2 directory")
    parser.add_argument("diagnostic_csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-cases", type=int, default=12)
    parser.add_argument("--max-image-offset-ms", type=float, default=100.0)
    args = parser.parse_args()

    with args.diagnostic_csv.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    # same-cell source stampに一致するmap行から、そのdepth frameのTF x/yも取得する。
    # map publish stampはdebug rateに間引かれており、source stampと違う場合があるので
    # map stamp単独ではposeを結び付けない。
    pose_by_cell_source = {
        (row["odom_cell_x"], row["odom_cell_y"], row["latest_source_stamp_ns"]): row
        for row in rows
    }
    # 同一セルの同一frameを一度だけ扱い、obstacle cueが大きい順で目視例を選ぶ。
    cases = [row for row in rows if row.get("fusion_mode") == "2"]
    cases.sort(key=lambda row: float(row.get("obstacle_m") or 0.0), reverse=True)
    cases = cases[:max(0, args.max_cases)]
    if not cases:
        raise SystemExit("fusion_mode=2の診断行がありません")

    targets = {}
    for index, row in enumerate(cases):
        for label, stamp_field in (
            ("old", "previous_ground_input_stamp_ns"),
            ("new", "last_accepted_ground_input_stamp_ns"),
        ):
            stamp = row.get(stamp_field, "")
            if stamp:
                targets[(index, label)] = int(stamp)

    # 必要な時刻の画像だけ保持し、bag内の全画像をRAMへ溜めない。
    rclpy.init()
    nearest = {}
    camera_info = {}
    nearest_imu = {}
    maximum_ns = int(args.max_image_offset_ms * 1e6)
    try:
        for mcap_path in sorted(args.bag.glob("*.mcap")):
            with mcap_path.open("rb") as stream:
                for _, channel, record in make_reader(stream).iter_messages(
                    topics=list(IMAGE_TYPES) + list(INFO_TOPICS) + [IMU_TOPIC]
                ):
                    topic = channel.topic
                    if topic in IMAGE_TYPES:
                        kind, message_type = IMAGE_TYPES[topic]
                    elif topic in INFO_TOPICS:
                        kind, message_type = "camera_info", CameraInfo
                    elif topic == IMU_TOPIC:
                        kind, message_type = "imu", Imu
                    else:
                        continue
                    message = deserialize_message(record.data, message_type)
                    current_stamp = stamp_ns(message)
                    if kind == "camera_info":
                        camera_info[topic] = {
                            "stamp_ns": current_stamp,
                            "width": message.width,
                            "height": message.height,
                            "frame_id": message.header.frame_id,
                            "k": list(message.k),
                        }
                        continue
                    if kind == "imu":
                        for key, target_stamp in targets.items():
                            delta = abs(current_stamp - target_stamp)
                            if delta <= maximum_ns and (
                                key not in nearest_imu or delta < nearest_imu[key][0]
                            ):
                                nearest_imu[key] = (
                                    delta, current_stamp,
                                    rpy_degrees(message.orientation),
                                )
                        continue
                    # 旧・新source stamp双方に最も近い画像frameをそれぞれ記録する。
                    for key, target_stamp in targets.items():
                        delta = abs(current_stamp - target_stamp)
                        match_key = (key, kind)
                        if delta <= maximum_ns and (
                            match_key not in nearest or delta < nearest[match_key][0]
                        ):
                            nearest[match_key] = (
                                delta, current_stamp, image_to_array(message, kind),
                                message.encoding, message.header.frame_id,
                            )
    finally:
        rclpy.shutdown()

    args.output.mkdir(parents=True, exist_ok=True)
    metadata = {"cases": [], "camera_info": camera_info,
                "pixel_overlay_warning": "RGBとdepthの画像幾何が一致することを保証しない。pixel markerはdepth画像上でのみ厳密。"}
    contact_rows = []
    for index, row in enumerate(cases):
        case = {
            "cell": [int(row["odom_cell_x"]), int(row["odom_cell_y"])],
            "map_stamp_ns": int(row["map_stamp_ns"]),
            "obstacle_m": float(row["obstacle_m"]),
            "ground_before_m": float(row["ground_before_fusion_m"]),
            "ground_after_m": float(row["ground_after_fusion_m"]),
            "relative_before_m": float(row["relative_ground_before_m"]),
            "relative_after_m": float(row["relative_ground_after_m"]),
            "sources": {},
        }
        tiles = []
        for label, prefix in (("old", "previous_ground_input_"),
                              ("new", "last_accepted_ground_input_")):
            source = {
                "stamp_ns": int(row[prefix + "stamp_ns"]),
                "pixel_uv": [int(row[prefix + "pixel_u"]), int(row[prefix + "pixel_v"])],
                "depth_m": float(row[prefix + "depth_m"]),
                "base_pose": {
                    "z_m": float(row[prefix + "base_z_m"]),
                    "roll_deg": float(row[prefix + "roll_deg"]),
                    "pitch_deg": float(row[prefix + "pitch_deg"]),
                    "yaw_deg": float(row[prefix + "yaw_deg"]),
                },
                "images": {},
            }
            source_pose_row = pose_by_cell_source.get((
                str(case["cell"][0]), str(case["cell"][1]),
                str(source["stamp_ns"]),
            ))
            if source_pose_row is not None:
                source["base_pose"].update({
                    "x_m": float(source_pose_row["capture_base_x_m"]),
                    "y_m": float(source_pose_row["capture_base_y_m"]),
                    "z_m": float(source_pose_row["capture_base_z_m"]),
                    "roll_deg": float(source_pose_row["capture_roll_deg"]),
                    "pitch_deg": float(source_pose_row["capture_pitch_deg"]),
                    "yaw_deg": float(source_pose_row["capture_yaw_deg"]),
                    "pose_stamp_ns": source["stamp_ns"],
                    "pose_stamp_source": "same-stamp forensic map row",
                })
            key = (index, label)
            if key in nearest_imu:
                delta, imu_stamp, imu_rpy = nearest_imu[key]
                source["nearest_wit_imu"] = {
                    "header_stamp_ns": imu_stamp,
                    "offset_ms": delta / 1e6,
                    "rpy_deg": imu_rpy,
                    "capture_minus_imu_rpy_deg": [
                        source["base_pose"][axis] - imu_rpy[i]
                        for i, axis in enumerate(("roll_deg", "pitch_deg", "yaw_deg"))
                    ],
                }
            depth_match = nearest.get((key, "depth"))
            color_match = nearest.get((key, "color"))
            marker = tuple(source["pixel_uv"])
            for kind, match in (("depth", depth_match), ("color", color_match)):
                if match is None:
                    continue
                delta, actual_stamp, array, encoding, frame_id = match
                output_image = depth_display(array) if kind == "depth" else array.copy()
                marker_used = False
                u, v = marker
                if 0 <= u < output_image.shape[1] and 0 <= v < output_image.shape[0]:
                    cv2.drawMarker(output_image, (u, v), (0, 0, 255),
                                   cv2.MARKER_CROSS, 25, 2)
                    cv2.putText(output_image, label.upper(), (max(0, u + 10), max(22, v - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
                                cv2.LINE_AA)
                    marker_used = True
                cv2.putText(output_image,
                            "{} {} offset={:.1f}ms".format(label, kind, delta / 1e6),
                            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (255, 255, 255), 2, cv2.LINE_AA)
                filename = "case{:02d}_{}_{}.png".format(index + 1, label, kind)
                cv2.imwrite(str(args.output / filename), output_image)
                source["images"][kind] = {
                    "file": filename,
                    "header_stamp_ns": actual_stamp,
                    "offset_ms": delta / 1e6,
                    "encoding": encoding,
                    "frame_id": frame_id,
                    "width": int(array.shape[1]),
                    "height": int(array.shape[0]),
                    "source_pixel_marker_drawn": marker_used,
                }
                if kind == "depth":
                    # 16-bit PNGは距離値をmm単位のまま保つため、視覚化色画像と別保存する。
                    raw_filename = "case{:02d}_{}_depth_mm.png".format(index + 1, label)
                    cv2.imwrite(str(args.output / raw_filename), array)
                    source["depth_neighborhood"] = depth_patch_statistics(
                        array, marker[0], marker[1]
                    )
                    source["images"]["depth_raw_mm"] = {"file": raw_filename}
                tiles.append(cv2.resize(output_image, (480, 300)))
            case["sources"][label] = source
        # 各caseの4画像（旧RGB/depth、新RGB/depth）を1行に並べる。
        if tiles:
            while len(tiles) < 4:
                tiles.append(np.zeros_like(tiles[0]))
            contact_rows.append(np.hstack(tiles[:4]))
        metadata["cases"].append(case)

    if contact_rows:
        cv2.imwrite(str(args.output / "ground_source_contact_sheet.png"),
                    np.vstack(contact_rows))
    (args.output / "ground_source_image_matches.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("置換例数:", len(cases))
    print("画像出力:", args.output)


if __name__ == "__main__":
    main()
