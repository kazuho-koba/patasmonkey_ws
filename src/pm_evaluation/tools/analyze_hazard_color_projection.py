#!/usr/bin/env python3
"""走行時hazard黒地点のdepth支持画素を、同時刻のRGB previewへ投影する。

7月bagにはCameraInfoが保存されていないため、既知のEEPROM Kから作ったdepth画像用Kと、
RGB preview用Kを明示的に指定する。どちらも640x400で同じ光学カメラ座標を使うが、
preview生成時の横方向stretchが異なる。その差をKの比で補正してからdepth画素をRGBへ描く。
bag内の全RGB画像は一度だけ走査し、診断対象時刻に近い画像だけをメモリへ保持する。
"""

import argparse
import bisect
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
COLOR_K = (516.8594971, 574.2882690, 351.2757568, 215.2326202)
DEFAULT_DEPTH_K = (574.28826904, 574.28826904, 354.75085449, 215.23262024)


def read_csv(path):
    """UTF-8 CSVを辞書行として読み込む。診断対象は数十セル程度に限定する。"""
    with Path(path).open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def image_to_bgr(message):
    """ROS Imageのrow stepと一般的なRGB/BGR encodingを考慮して画像viewを作る。"""
    encoding = message.encoding.lower()
    channels = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(encoding)
    if channels is None:
        raise ValueError("未対応のcolor encoding: " + encoding)
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


def parse_target_rows(path_matches, supports, max_targets):
    """経路上黒セルの時刻・cellに一致する平面supportを抽出し、高低端を選ぶ。"""
    by_target = {}
    # mapperが保存した生CSVと解析済みsupport CSVの双方で、1行の意味を揃える。
    for raw_row in supports:
        row = dict(raw_row)
        row["source_stamp_ns"] = raw_row.get(
            "source_stamp_ns", raw_row.get("latest_source_stamp_ns", ""))
        row["source_pixel_u"] = raw_row.get(
            "source_pixel_u", raw_row.get("frame_min_pixel_u", ""))
        row["source_pixel_v"] = raw_row.get(
            "source_pixel_v", raw_row.get("frame_min_pixel_v", ""))
        row["source_depth_m"] = raw_row.get(
            "source_depth_m", raw_row.get("frame_min_axial_depth_m", ""))
        row["source_fusion_mode"] = raw_row.get(
            "source_fusion_mode", raw_row.get("fusion_mode", ""))
        row["map_relative_elevation_m"] = raw_row.get(
            "map_relative_elevation_m", raw_row.get("relative_elevation_m", ""))
        row["source_frame_sample_count"] = raw_row.get(
            "source_frame_sample_count", raw_row.get("frame_sample_count", ""))
        key = (int(row["map_stamp_ns"]), int(row["target_cell_x"]),
               int(row["target_cell_y"]))
        by_target.setdefault(key, []).append(row)

    targets = []
    for index, path_row in enumerate(path_matches[:max_targets], start=1):
        stamp = int(path_row["map_stamp_ns"])
        cell_x, cell_y = int(path_row["odom_cell_x"]), int(path_row["odom_cell_y"])
        rows = by_target.get((stamp, cell_x, cell_y), [])
        # 平面支持セルから、map elevationが最も低い点と高い点を選ぶ。
        # source pixelはそのセルの直近観測由来なので、色画像上の表面候補を確認しやすい。
        valid = [row for row in rows if row.get("map_relative_elevation_m") not in (None, "")
                 and row.get("source_pixel_u") not in (None, "")]
        selected = []
        if valid:
            selected = [min(valid, key=lambda row: float(row["map_relative_elevation_m"])),
                        max(valid, key=lambda row: float(row["map_relative_elevation_m"]))]
            if (selected[0]["source_stamp_ns"], selected[0]["support_cell_x"],
                    selected[0]["support_cell_y"]) == (
                    selected[1]["source_stamp_ns"], selected[1]["support_cell_x"],
                    selected[1]["support_cell_y"]):
                selected = selected[:1]
        targets.append({"index": index, "path": path_row, "supports": rows,
                        "selected": selected})
    return targets


def target_source_stamps(targets):
    """高低端として選んだ支持セルのsource stampだけをRGB検索対象にする。"""
    return sorted({int(row["source_stamp_ns"])
                   for target in targets for row in target["selected"]})


def nearest_target_index(stamp_ns, targets):
    """時刻順targetsの中から最も近いstamp indexを返す。"""
    index = bisect.bisect_left(targets, stamp_ns)
    candidates = [candidate for candidate in (index - 1, index)
                  if 0 <= candidate < len(targets)]
    return min(candidates, key=lambda candidate: abs(targets[candidate] - stamp_ns))


def extract_nearest_color_frames(bag_dir, stamps, max_offset_ms):
    """MCAPのRGB topicを一度走査し、要求stampごとの最寄りframeだけ保持する。"""
    if not stamps:
        return {}
    best = {}
    max_delta_ns = int(max_offset_ms * 1e6)
    rclpy.init()
    try:
        # metadata順と無関係に全分割bagを走査する。ROS image payloadはRGBだけdeserializeする。
        for mcap_path in sorted(Path(bag_dir).glob("*.mcap")):
            with mcap_path.open("rb") as stream:
                reader = make_reader(stream)
                for _, _, record in reader.iter_messages(topics=[COLOR_TOPIC]):
                    message = deserialize_message(record.data, Image)
                    stamp = (message.header.stamp.sec * 1_000_000_000
                             + message.header.stamp.nanosec)
                    index = nearest_target_index(stamp, stamps)
                    target_stamp = stamps[index]
                    delta = abs(stamp - target_stamp)
                    if delta > max_delta_ns:
                        continue
                    previous = best.get(target_stamp)
                    if previous is None or delta < previous[0]:
                        best[target_stamp] = (delta, stamp, message.width,
                                              message.height, message.header.frame_id,
                                              image_to_bgr(message))
    finally:
        rclpy.shutdown()
    return best


def depth_pixel_to_color(u_depth, v_depth, depth_k):
    """共通RGB光学rayをdepth出力KからRGB preview Kへ座標変換する。"""
    fx_d, fy_d, cx_d, cy_d = depth_k
    fx_c, fy_c, cx_c, cy_c = COLOR_K
    return (fx_c * (u_depth - cx_d) / fx_d + cx_c,
            fy_c * (v_depth - cy_d) / fy_d + cy_c)


def draw_source_overlay(image, rows, source_stamp, target, depth_k):
    """一つのdepth source frame/cellのstride samplesをRGB上に描き込む。"""
    output = image.copy()
    colors = [(40, 30, 240), (20, 220, 250), (240, 80, 30), (220, 40, 220)]
    source_cells = {(int(row["support_cell_x"]), int(row["support_cell_y"])): row
                    for row in target["selected"]
                    if int(row["source_stamp_ns"]) == source_stamp}
    for pixel in rows:
        if int(pixel["source_stamp_ns"]) != source_stamp:
            continue
        cell = (int(pixel["odom_cell_x"]), int(pixel["odom_cell_y"]))
        if cell not in source_cells:
            continue
        u_color, v_color = depth_pixel_to_color(
            float(pixel["pixel_u"]), float(pixel["pixel_v"]), depth_k)
        u, v = int(round(u_color)), int(round(v_color))
        if not (0 <= u < output.shape[1] and 0 <= v < output.shape[0]):
            continue
        # 同一support cellは一定色。4x間引きの各depth sampleを小点で表示する。
        cell_order = sorted(source_cells).index(cell)
        color = colors[cell_order % len(colors)]
        cv2.circle(output, (u, v), 2, color, -1, cv2.LINE_AA)

    # plane_supportに記録したmin pixelを二重丸にし、対象高さと観測ageを付記する。
    for cell, row in source_cells.items():
        u, v = depth_pixel_to_color(float(row["source_pixel_u"]),
                                    float(row["source_pixel_v"]), depth_k)
        u, v = int(round(u)), int(round(v))
        if 0 <= u < output.shape[1] and 0 <= v < output.shape[0]:
            cv2.drawMarker(output, (u, v), (255, 255, 255), cv2.MARKER_CROSS,
                           15, 2, cv2.LINE_AA)
            label = "cell {}:{} z={:.3f} age={:.2f}s".format(
                cell[0], cell[1], float(row["map_relative_elevation_m"]),
                float(row["age_s"]))
            cv2.putText(output, label, (max(2, u + 8), max(18, v - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1,
                        cv2.LINE_AA)
    cv2.putText(output, "black #{} source {:.3f}s rel to bag".format(
        target["index"], (source_stamp - int(target["path"]["map_stamp_ns"])) / 1e9),
        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
    return output


def summarize_target(target):
    """黒地点の支持時刻幅と各姿勢軸の変動幅を算出する。"""
    rows = target["supports"]
    stamps = [int(row["source_stamp_ns"]) for row in rows]
    result = {"index": target["index"],
              "map_stamp_ns": int(target["path"]["map_stamp_ns"]),
              "odom_cell_x": int(target["path"]["odom_cell_x"]),
              "odom_cell_y": int(target["path"]["odom_cell_y"]),
              "max_cause": int(target["path"]["max_cause"]),
              "slope_deg": float(target["path"]["slope_deg"]),
              "step_m": (float(target["path"]["step_m"])
                         if target["path"].get("step_m") else None),
              "obstacle_m": (float(target["path"]["obstacle_m"])
                             if target["path"].get("obstacle_m") else None),
              "support_count": len(rows),
              "support_source_frame_count": len(set(stamps)),
              "support_stamp_span_s": ((max(stamps) - min(stamps)) / 1e9 if stamps else None),
              "support_oldest_age_s": (
                  (int(target["path"]["map_stamp_ns"]) - min(stamps)) / 1e9
                  if stamps else None),
              "support_newest_age_s": (
                  (int(target["path"]["map_stamp_ns"]) - max(stamps)) / 1e9
                  if stamps else None),
              "base_z_span_m": _range(rows, "capture_base_z_m"),
              "roll_span_deg": _range(rows, "capture_roll_deg"),
              "pitch_span_deg": _range(rows, "capture_pitch_deg"),
              "support_map_height_range_m": _range(rows, "map_relative_elevation_m"),
              "selected_source_stamps_ns": [int(row["source_stamp_ns"])
                                            for row in target["selected"]]}
    return result


def _range(rows, key):
    """CSV数値列のmax-min。欠損値は無視する。"""
    values = [float(row[key]) for row in rows if row.get(key) not in (None, "")]
    return max(values) - min(values) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-targets", type=int, default=100)
    parser.add_argument("--max-color-offset-ms", type=float, default=100.0)
    parser.add_argument("--depth-k", type=float, nargs=4, default=DEFAULT_DEPTH_K,
                        metavar=("FX", "FY", "CX", "CY"))
    args = parser.parse_args()
    forensic = args.result_dir / "forensic"
    analysis = args.result_dir / "full_frame_analysis"
    targets = parse_target_rows(
        read_csv(analysis / "lookahead_black_path_matches.csv"),
        read_csv(forensic / "plane_support.csv"), args.max_targets)
    pixels = read_csv(forensic / "frame_pixels.csv")
    stamps = target_source_stamps(targets)
    colors = extract_nearest_color_frames(args.bag, stamps, args.max_color_offset_ms)
    args.output.mkdir(parents=True, exist_ok=True)

    summaries, tiles = [], []
    unmatched = []
    for target in targets:
        summaries.append(summarize_target(target))
        target_tiles = []
        for selected_index, support in enumerate(target["selected"]):
            stamp = int(support["source_stamp_ns"])
            color_record = colors.get(stamp)
            if color_record is None:
                unmatched.append({"target": target["index"], "source_stamp_ns": stamp})
                continue
            delta, color_stamp, width, height, frame_id, image = color_record
            overlay = draw_source_overlay(image, pixels, stamp, target, args.depth_k)
            cv2.putText(overlay, "RGB stamp delta={:.1f}ms frame={}".format(
                (color_stamp - stamp) / 1e6, frame_id), (8, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
            filename = "black{:02d}_support{}.png".format(target["index"], selected_index)
            cv2.imwrite(str(args.output / filename), overlay)
            summary = summaries[-1]
            summary.setdefault("images", []).append({
                "file": filename, "source_stamp_ns": stamp,
                "color_stamp_ns": color_stamp, "signed_delta_ms": (color_stamp - stamp) / 1e6,
                "width": width, "height": height, "frame_id": frame_id,
                "selected_support_cell": [int(support["support_cell_x"]),
                                          int(support["support_cell_y"])],
                "support_map_elevation_m": float(support["map_relative_elevation_m"]),
                "source_fusion_mode": int(support["source_fusion_mode"]),
                "depth_pixel": [int(support["source_pixel_u"]),
                                int(support["source_pixel_v"])],
                "depth_m": float(support["source_depth_m"]),
            })
            target_tiles.append(cv2.resize(overlay, (480, 300)))
        if target_tiles:
            while len(target_tiles) < 2:
                target_tiles.append(np.zeros_like(target_tiles[0]))
            tiles.append(np.vstack((target_tiles[0], target_tiles[1])))

    if tiles:
        # 各targetは高低supportを別画像に保ち、異時刻のrayを一枚へ誤重畳しない。
        cv2.imwrite(str(args.output / "color_projection_contact_sheet.png"),
                    np.vstack(tiles))
    report = {"bag": str(args.bag), "target_count": len(targets),
              "rgb_targets_matched": len(colors), "rgb_target_count": len(stamps),
              "depth_k": list(args.depth_k), "color_k": list(COLOR_K),
              "pixel_mapping": "u_rgb=fx_rgb*(u_depth-cx_depth)/fx_depth+cx_rgb; "
                              "v_rgb=fy_rgb*(v_depth-cy_depth)/fy_depth+cy_rgb",
              "targets": summaries, "unmatched_source_stamps": unmatched}
    (args.output / "color_projection_summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("黒地点:", len(targets), "RGB対応:", len(colors), "/", len(stamps),
          "source frame, 未対応:", len(unmatched))
    print("出力:", args.output)


if __name__ == "__main__":
    main()
