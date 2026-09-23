#!/usr/bin/env python3
"""hazard黒区間の代表地点に近いカラー/depth画像をbagから抽出する。

経路診断CSVの連続した中心black区間をcue組合せごとに選び、区間中点の「2 m手前map時刻」
と「実際の通過時刻」に近い画像だけを保存する。MCAP内の他topic/画像は保持しない。
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


IMAGE_TOPICS = {
    "/oak/color/image_raw": "color",
    "/oak/depth/image_raw": "depth",
}
CUES = ("slope", "roughness", "step", "obstacle")


def load_events(path, limit):
    """中心blackの連続runを作り、cue組合せごとに最長runを選ぶ。"""
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    runs, start = [], None
    for index, row in enumerate(rows + [{"center_hazard": ""}]):
        is_black = row.get("center_hazard") == "100"
        if is_black and start is None:
            start = index
        elif not is_black and start is not None:
            group = rows[start:index]
            midpoint = group[len(group) // 2]
            mask = tuple(
                cue for cue, column in zip(CUES, (
                    "center_slope_deg", "center_roughness_m",
                    "center_step_m", "center_obstacle_m"))
                if midpoint.get(column) not in (None, "")
                and float(midpoint[column]) >= (20.0 if cue == "slope" else
                    0.03 if cue == "roughness" else 0.07 if cue == "step" else 0.20)
            )
            runs.append({"start_index": start, "end_index": index - 1,
                         "sample_count": len(group), "mask": mask,
                         "midpoint": midpoint})
            start = None
    # Cue groupが同じrunを代表させる。最長runはcue重複を問わず必ず含める。
    by_mask = {}
    for run in runs:
        key = run["mask"]
        if key not in by_mask or run["sample_count"] > by_mask[key]["sample_count"]:
            by_mask[key] = run
    chosen = sorted(by_mask.values(), key=lambda item: item["sample_count"], reverse=True)
    if runs:
        longest = max(runs, key=lambda item: item["sample_count"])
        if longest not in chosen:
            chosen.insert(0, longest)
    return chosen[:limit]


def image_array(message, kind):
    """ROS Imageのstep/paddingとencodingを考慮してNumPy画像を作る。"""
    encoding = message.encoding.lower()
    if kind == "color":
        channels = 3 if encoding in ("rgb8", "bgr8") else 4 if encoding == "rgba8" else 0
        if not channels:
            raise ValueError("未対応のcolor encoding: " + encoding)
        view = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )[:, :message.width * channels].reshape(message.height, message.width, channels)
        if encoding == "rgb8":
            return cv2.cvtColor(view, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(view, cv2.COLOR_RGBA2BGR)
        return view.copy()
    dtype = ">u2" if message.is_bigendian else "<u2"
    row_words = message.step // 2
    return np.frombuffer(message.data, dtype=dtype).reshape(
        message.height, row_words
    )[:, :message.width].copy()


def depth_visual(depth_mm):
    """depth 0.4..4mを共通scaleのTurbo色mapへ変換する。"""
    metres = depth_mm.astype(np.float32) * 0.001
    valid = np.isfinite(metres) & (metres >= 0.4) & (metres <= 4.0)
    gray = np.zeros(depth_mm.shape, dtype=np.uint8)
    gray[valid] = np.rint((metres[valid] - 0.4) * (255.0 / 3.6)).astype(np.uint8)
    colored = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    colored[~valid] = 0
    return colored


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("path_csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-events", type=int, default=8)
    args = parser.parse_args()
    events = load_events(args.path_csv, args.max_events)
    if not events:
        raise SystemExit("CSVにhazard=100の中心区間がありません")
    # 各選択区間のmap時刻（通過前）とpassage時刻に対する画像header stampを探す。
    targets = []
    for event_index, event in enumerate(events):
        row = event["midpoint"]
        for phase, column in (("lookahead", "lookahead_map_time_ns"),
                              ("passage", "passage_time_ns")):
            stamp = row.get(column)
            if stamp not in (None, ""):
                targets.append({"event": event_index, "phase": phase,
                                "stamp_ns": int(stamp)})
    rclpy.init()
    best = {}
    try:
        for mcap_path in sorted(args.bag.glob("*.mcap")):
            with mcap_path.open("rb") as stream:
                for _, channel, record in make_reader(stream).iter_messages(
                        topics=list(IMAGE_TOPICS)):
                    kind = IMAGE_TOPICS[channel.topic]
                    message = deserialize_message(record.data, Image)
                    stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                    # 画像は約10Hzなので、周辺targetだけを比較対象にする。
                    for target in targets:
                        delta = abs(stamp - target["stamp_ns"])
                        if delta > 400_000_000:
                            continue
                        key = (target["event"], target["phase"], kind)
                        if key not in best or delta < best[key][0]:
                            best[key] = (delta, stamp, image_array(message, kind))
    finally:
        rclpy.shutdown()

    args.output.mkdir(parents=True, exist_ok=True)
    metadata = []
    tiles = []
    for event_index, event in enumerate(events):
        row = event["midpoint"]
        event_info = {"event": event_index + 1, "cue_mask": list(event["mask"]),
                      "sample_count": event["sample_count"],
                      "x_odom_m": float(row["x_odom_m"]),
                      "y_odom_m": float(row["y_odom_m"]),
                      "map_stamp_ns": int(row["lookahead_map_time_ns"]),
                      "passage_stamp_ns": int(row["passage_time_ns"]),
                      "images": {}}
        phase_tiles = []
        for phase in ("lookahead", "passage"):
            for kind in ("color", "depth"):
                key = (event_index, phase, kind)
                if key not in best:
                    continue
                delta, actual_stamp, array = best[key]
                shown = array if kind == "color" else depth_visual(array)
                title = "#{} {} {} Δ{:.0f}ms".format(
                    event_index + 1, phase, kind, delta / 1e6)
                cv2.putText(shown, title, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2, cv2.LINE_AA)
                filename = "event{:02d}_{}_{}.png".format(event_index + 1, phase, kind)
                cv2.imwrite(str(args.output / filename), shown)
                event_info["images"][phase + "_" + kind] = {
                    "file": filename, "header_stamp_ns": actual_stamp,
                    "offset_ms": delta / 1e6}
                phase_tiles.append(cv2.resize(shown, (480, 300)))
        event_info["images_found"] = len(phase_tiles)
        metadata.append(event_info)
        if phase_tiles:
            # 2x2: 通過前RGB/depthと、通過時RGB/depthを一行へ配置する。
            while len(phase_tiles) < 4:
                phase_tiles.append(np.zeros_like(phase_tiles[0]))
            tiles.append(np.vstack((np.hstack(phase_tiles[:2]),
                                    np.hstack(phase_tiles[2:4]))))
    if tiles:
        cv2.imwrite(str(args.output / "contact_sheet.png"), np.vstack(tiles))
    (args.output / "image_matches.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("抽出イベント数:", len(metadata), "画像:", sum(x["images_found"] for x in metadata))


if __name__ == "__main__":
    main()
