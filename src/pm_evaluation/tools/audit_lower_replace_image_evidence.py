#!/usr/bin/env python3
"""lower-replace ground候補9件をbagのRGB/depth画像と対応づけて監査する。

前段で作成したsupport_fusion_history_all15.jsonからmode 2の観測を抽出し、同一
odom cellに直前採用されたground候補と、新たに低い候補として採用されたframeを比べる。
元depth pixelは深度画像に正確に描画し、RGB画像にはEEPROM intrinsics比による近似位置を
参考表示する。depth近傍統計とframe内セルのmin/maxもJSONへ保存する。

このツールはオフライン診断専用。7月bagにCameraInfoがなく、color/depth外部校正や完全な
時刻同期もないため、RGB markerと色画像のpixel一致、あるいは可視景物からground truthを
証明するものではない。frameごとの画像だけを保持し、bag全画像をメモリに積まない。
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
# 7月bagにCameraInfoがないので、前段診断と同じEEPROM intrinsicsを利用する。
COLOR_K = (516.8594971, 574.2882690, 351.2757568, 215.2326202)
DEPTH_K = (574.28826904, 574.28826904, 354.75085449, 215.23262024)


def read_csv(path):
    """UTF-8のforensic CSVを辞書行として読む。対象行だけを後段で利用する。"""
    with Path(path).open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def image_stamp(message):
    """sensor_msgs/Imageのheader stampを整数nanosecondへ変換する。"""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def color_array(message):
    """ROS Imageのstep/paddingを守り、対応RGB encodingをBGR配列へ変換する。"""
    encoding = message.encoding.lower()
    channels = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(encoding)
    if channels is None:
        raise ValueError("未対応color encoding: " + message.encoding)
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step)[:, :message.width * channels]
    raw = raw.reshape(message.height, message.width, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(raw, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
    return raw.copy()


def depth_array(message):
    """16-bit millimetre depthをrow stepとendiannessを保って読み込む。"""
    if message.encoding not in ("16UC1", "mono16"):
        raise ValueError("未対応depth encoding: " + message.encoding)
    dtype = ">u2" if message.is_bigendian else "<u2"
    rows = np.frombuffer(message.data, dtype=dtype).reshape(
        message.height, message.step // 2)
    return rows[:, :message.width].copy()


def collect_lower_replace_pairs(fusion_json, frame_events_csv):
    """mode 2ごとに、同一cellの直前accepted候補と対応frame eventを選ぶ。"""
    fusion = json.loads(Path(fusion_json).read_text(encoding="utf-8"))
    events = {
        (int(row["source_stamp_ns"]), int(row["odom_cell_x"]),
         int(row["odom_cell_y"])): row
        for row in read_csv(frame_events_csv)
    }
    pairs = []
    for target in fusion["targets"]:
        for cell in target["support_cells"]:
            previous = None
            history = cell["observations_before_target_map"]
            for current in history:
                # 初期化・lower-replace・mergeはいずれも採用候補。mode 0は既存groundを
                # 維持するのでprevious accepted sourceを更新しない。
                if current["fusion_mode"] == 2 and previous is not None:
                    xy = cell["cell"]
                    old_event = events.get((previous["stamp_ns"], xy[0], xy[1]))
                    new_event = events.get((current["stamp_ns"], xy[0], xy[1]))
                    if old_event is None or new_event is None:
                        raise RuntimeError("mode 2候補のframe_eventsが見つかりません")
                    pairs.append({
                        "target_index": target["target_index"], "cell": xy,
                        "old": {**previous, "frame_event": old_event},
                        "new": {**current, "frame_event": new_event},
                    })
                if current["fusion_mode"] in (1, 2, 3):
                    previous = current
    pairs.sort(key=lambda pair: pair["new"]["stamp_ns"])
    return pairs


def extract_requested_frames(bag_dir, stamps, color_tolerance_ms):
    """指定depth frameと最近傍color frameだけをMCAP走査中に選んで保持する。"""
    import rclpy
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    requested = sorted(set(stamps))
    images = {stamp: {} for stamp in requested}
    best_color = {}
    tolerance_ns = int(color_tolerance_ms * 1e6)
    rclpy.init()
    try:
        for path in sorted(Path(bag_dir).glob("*.mcap")):
            with path.open("rb") as stream:
                reader = make_reader(stream)
                for _, channel, record in reader.iter_messages(
                        topics=[COLOR_TOPIC, DEPTH_TOPIC]):
                    topic = channel.topic if hasattr(channel, "topic") else channel
                    message = deserialize_message(record.data, Image)
                    stamp = image_stamp(message)
                    if topic == DEPTH_TOPIC:
                        if stamp in images:
                            images[stamp]["depth"] = {
                                "stamp_ns": stamp, "array": depth_array(message),
                                "frame_id": message.header.frame_id,
                            }
                        continue
                    # color stampを前後する要求depth stampへ対応づけ、最も近い一枚だけ残す。
                    insertion = bisect.bisect_left(requested, stamp)
                    candidates = [i for i in (insertion - 1, insertion)
                                  if 0 <= i < len(requested)]
                    for index in candidates:
                        target = requested[index]
                        delta = abs(stamp - target)
                        if delta > tolerance_ns:
                            continue
                        if target not in best_color or delta < best_color[target][0]:
                            best_color[target] = (delta, {
                                "stamp_ns": stamp, "array": color_array(message),
                                "frame_id": message.header.frame_id,
                            })
    finally:
        rclpy.shutdown()
    for stamp, (delta, frame) in best_color.items():
        images[stamp]["color"] = {**frame, "offset_ms": delta / 1e6}
    return images


def depth_to_color(pixel):
    """共通pinhole rayを仮定してdepth pixelをcolor previewへ近似投影する。"""
    u, v = pixel
    fxd, fyd, cxd, cyd = DEPTH_K
    fxc, fyc, cxc, cyc = COLOR_K
    return np.asarray([fxc * (u - cxd) / fxd + cxc,
                       fyc * (v - cyd) / fyd + cyc], dtype=np.float64)


def depth_patch(depth_mm, pixel, radius):
    """選択画素まわりのvalid depth分布を要約する。"""
    u, v = pixel
    y0, y1 = max(0, v - radius), min(depth_mm.shape[0], v + radius + 1)
    x0, x1 = max(0, u - radius), min(depth_mm.shape[1], u + radius + 1)
    patch = depth_mm[y0:y1, x0:x1].astype(np.float32) * 0.001
    valid = patch[(patch >= 0.4) & (patch <= 6.0)]
    if valid.size == 0:
        return {"radius_px": radius, "valid_count": 0}
    return {
        "radius_px": radius, "valid_count": int(valid.size),
        "min_m": float(valid.min()), "p10_m": float(np.percentile(valid, 10)),
        "median_m": float(np.median(valid)), "p90_m": float(np.percentile(valid, 90)),
        "max_m": float(valid.max()), "std_m": float(valid.std()),
    }


def colorize_depth(depth_mm):
    """全イベントで共通の0.4〜5m色スケールを使って距離を表示する。"""
    metres = depth_mm.astype(np.float32) * 0.001
    valid = (metres >= 0.4) & (metres <= 5.0)
    gray = np.zeros(depth_mm.shape, dtype=np.uint8)
    gray[valid] = np.clip((metres[valid] - 0.4) * (255.0 / 4.6), 0, 255).astype(
        np.uint8)
    result = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    result[~valid] = (20, 20, 20)
    return result


def annotate(image, pixel, title, color, radius=8):
    """画素位置と近傍領域を示し、パネル名を付ける。"""
    canvas = image.copy()
    u, v = map(int, pixel)
    cv2.rectangle(canvas, (max(0, u-radius), max(0, v-radius)),
                  (min(canvas.shape[1]-1, u+radius),
                   min(canvas.shape[0]-1, v+radius)), color, 1, cv2.LINE_AA)
    cv2.drawMarker(canvas, (u, v), color, cv2.MARKER_CROSS, 17, 2, cv2.LINE_AA)
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(canvas, title, (6, 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.47, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def inspect_source(source, images):
    """片方のsource frameについてpixel depth、局所patch、pose provenanceを返す。"""
    stamp = source["stamp_ns"]
    if stamp not in images or "depth" not in images[stamp]:
        raise RuntimeError("depth frameがbag内にありません: {}".format(stamp))
    depth_frame = images[stamp]["depth"]
    event = source["frame_event"]
    pixel = [source["pixel_u"], source["pixel_v"]]
    measured = float(depth_frame["array"][pixel[1], pixel[0]]) * 0.001
    output = {
        "stamp_ns": stamp, "pixel_uv": pixel,
        "candidate_world_xyz_m": source["candidate_world_xyz_m"],
        "candidate_world_z_m": source["candidate_world_z_m"],
        "candidate_depth_recorded_m": source["depth_m"],
        "depth_image_at_pixel_m": measured,
        "pixel_depth_match_m": measured - source["depth_m"],
        "patch_3x3": depth_patch(depth_frame["array"], pixel, 1),
        "patch_17x17": depth_patch(depth_frame["array"], pixel, 8),
        "camera_translation_xyz_m": source["camera_translation_xyz_m"],
        "base_roll_pitch_yaw_deg": [source["base_roll_deg"],
                                     source["base_pitch_deg"],
                                     source["base_yaw_deg"]],
        "ground_before_m": source["ground_before_m"],
        "ground_after_m": source["ground_after_m"],
        "fusion_mode": source["fusion_mode"],
        "frame_min_world_z_m": float(event["frame_min_world_z_m"]),
        "frame_max_world_z_m": float(event["frame_max_world_z_m"]),
        "frame_cell_world_z_span_m": float(event["frame_max_world_z_m"])
                                      - float(event["frame_min_world_z_m"]),
        "frame_sample_count": int(event["sample_count"]),
        "camera_frame_id": depth_frame["frame_id"],
        "color_pixel_approx_uv": depth_to_color(pixel).tolist(),
    }
    if "color" in images[stamp]:
        color = images[stamp]["color"]
        output["nearest_color_stamp_ns"] = color["stamp_ns"]
        output["color_offset_ms"] = color["offset_ms"]
        output["color_frame_id"] = color["frame_id"]
    else:
        output["nearest_color_stamp_ns"] = None
        output["color_offset_ms"] = None
    return output


def make_case_image(pair, old_metrics, new_metrics, images, output_path):
    """旧/新candidateのRGBとdepth画面を2x2に並べた目視用画像を作る。"""
    panels = []
    for label, source, metrics in (("old", pair["old"], old_metrics),
                                   ("new", pair["new"], new_metrics)):
        stamp = source["stamp_ns"]
        frame = images[stamp]
        depth = frame["depth"]["array"]
        uv = [source["pixel_u"], source["pixel_v"]]
        depth_panel = annotate(colorize_depth(depth), uv,
                               "{} depth exact pixel {} depth={:.3f}m".format(
                                   label, uv, source["depth_m"]), (0, 0, 255))
        if "color" in frame:
            color_uv = np.rint(metrics["color_pixel_approx_uv"]).astype(int).tolist()
            color_panel = annotate(frame["color"]["array"], color_uv,
                                   "{} RGB approximate marker; offset={:.1f}ms".format(
                                       label, frame["color"]["offset_ms"]),
                                   (255, 255, 255))
        else:
            color_panel = np.zeros_like(depth_panel)
            cv2.putText(color_panel, "{} RGB frame unavailable".format(label),
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 2)
        panels.append((color_panel, depth_panel))
    canvas = np.vstack((np.hstack(panels[0]), np.hstack(panels[1])))
    delta_z = pair["new"]["candidate_world_z_m"] - pair["old"]["candidate_world_z_m"]
    title = ("target {} cell {} | candidate dz={:+.3f}m | ground {:.3f}->{:.3f}m"
             .format(pair["target_index"], pair["cell"], delta_z,
                     pair["new"]["ground_before_m"], pair["new"]["ground_after_m"]))
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 32), (0, 0, 0), -1)
    cv2.putText(canvas, title, (8, 23), cv2.FONT_HERSHEY_SIMPLEX,
                0.62, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(output_path), canvas)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("fusion_history_json", type=Path)
    parser.add_argument("frame_events_csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--color-tolerance-ms", type=float, default=80.0)
    args = parser.parse_args()

    pairs = collect_lower_replace_pairs(args.fusion_history_json, args.frame_events_csv)
    if not pairs:
        raise RuntimeError("lower-replace mode 2のpairがありません")
    requested_stamps = [source["stamp_ns"] for pair in pairs
                        for source in (pair["old"], pair["new"])]
    images = extract_requested_frames(args.bag, requested_stamps,
                                      args.color_tolerance_ms)
    args.output.mkdir(parents=True, exist_ok=True)
    outputs = []
    for number, pair in enumerate(pairs, start=1):
        old_metrics = inspect_source(pair["old"], images)
        new_metrics = inspect_source(pair["new"], images)
        delta_xyz = (np.asarray(pair["new"]["candidate_world_xyz_m"])
                     - np.asarray(pair["old"]["candidate_world_xyz_m"]))
        record = {
            "case": number, "target_index": pair["target_index"],
            "odom_cell": pair["cell"],
            "candidate_delta_xyz_m": delta_xyz.tolist(),
            "candidate_delta_xy_m": float(np.linalg.norm(delta_xyz[:2])),
            "candidate_delta_z_m": float(delta_xyz[2]),
            "lower_replace_drop_m": (pair["new"]["ground_before_m"]
                                     - pair["new"]["ground_after_m"]),
            "old": old_metrics, "new": new_metrics,
        }
        filename = "lower_replace_{:02d}_target{:02d}_cell{}_{}.png".format(
            number, pair["target_index"], pair["cell"][0], pair["cell"][1])
        make_case_image(pair, old_metrics, new_metrics, images,
                        args.output / filename)
        record["overlay"] = filename
        outputs.append(record)

    payload = {
        "bag": str(args.bag), "color_k": COLOR_K, "depth_k": DEPTH_K,
        "lower_replace_cases": outputs,
        "limitations": [
            "RGB markerは7月bagのCameraInfo不在のためK比による近似で、外部registrationではない",
            "RGBとdepthのheader時刻は異なり、nearest color frameを使う",
            "局所depth patchは地面・縁石・物体を混ぜる場合がある",
            "画像とdepthの局所統計だけではpose/depthのどちらが真値から外れたか確定しない",
        ],
    }
    out_json = args.output / "lower_replace_image_audit.json"
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print("lower-replace events:", len(outputs), "depth frames:",
          sum("depth" in images.get(stamp, {}) for stamp in set(requested_stamps)),
          "color matches:", sum("color" in images.get(stamp, {})
                                 for stamp in set(requested_stamps)))
    for row in outputs:
        print("case", row["case"], "target", row["target_index"],
              "cell", row["odom_cell"], "dxy", round(row["candidate_delta_xy_m"], 3),
              "dz", round(row["candidate_delta_z_m"], 3),
              "depth old/new", row["old"]["candidate_depth_recorded_m"],
              row["new"]["candidate_depth_recorded_m"],
              "RGB offsets", row["old"]["color_offset_ms"],
              row["new"]["color_offset_ms"])
    print("saved:", out_json)


if __name__ == "__main__":
    main()
