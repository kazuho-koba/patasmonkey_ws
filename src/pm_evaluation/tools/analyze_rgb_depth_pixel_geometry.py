#!/usr/bin/env python3
"""RGB previewとRGB整列depthのK/D変換差をtracked feature上で数値化する。

7月bagにはCameraInfoがないため、過去に実機EEPROMから確認したRGB/depth KとDを
入力として使う。EEPROM Dを含む正確なpixel warpと、従来使ったKのみの線形warp、
画素indexを同一視する仮定の差を、保存済みの舗装特徴track位置で比較する。
これは画像上の独立ground truthではなく、driverのresize仕様に対する幾何計算である。
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


COLOR_K = np.array([[516.8594971, 0.0, 351.2757568],
                    [0.0, 574.2882690, 215.2326202],
                    [0.0, 0.0, 1.0]], dtype=np.float64)
DEPTH_K = np.array([[574.28826904, 0.0, 354.75085449],
                    [0.0, 574.28826904, 215.23262024],
                    [0.0, 0.0, 1.0]], dtype=np.float64)
# 7月bagに対応すると過去に実機EEPROMで照合したRGB rational-polynomial D。
DISTORTION_D = np.array([10.22323513, -109.25149536, -0.00044202,
                         0.00103162, 320.91735840, 9.92284966,
                         -107.20381165, 315.32150269], dtype=np.float64)


def percentiles(values):
    """pixel距離列の中央値・p90・最大値を返す。"""
    values = np.asarray(values, dtype=np.float64)
    return {"p50_px": float(np.percentile(values, 50)),
            "p90_px": float(np.percentile(values, 90)),
            "max_px": float(np.max(values))}


def map_pixels(rgb_pixels):
    """RGB画素をKのみ/ K+Dでdepth出力画素へ写し、誤ったidentity案も比較する。"""
    pixels = np.asarray(rgb_pixels, dtype=np.float64).reshape(-1, 1, 2)
    fx_c, fy_c = COLOR_K[0, 0], COLOR_K[1, 1]
    cx_c, cy_c = COLOR_K[0, 2], COLOR_K[1, 2]
    fx_d, fy_d = DEPTH_K[0, 0], DEPTH_K[1, 1]
    cx_d, cy_d = DEPTH_K[0, 2], DEPTH_K[1, 2]
    # Kのみの線形warp。以前の画像投影解析で使用した式と同じ。
    pinhole = np.column_stack((
        fx_d * (pixels[:, 0, 0] - cx_c) / fx_c + cx_d,
        fy_d * (pixels[:, 0, 1] - cy_c) / fy_c + cy_d,
    ))
    # RGB画像の歪みpixelを理想rayへ戻し、depth側のK/Dで再投影する。
    normalized = cv2.undistortPoints(pixels, COLOR_K, DISTORTION_D)
    rays = np.column_stack((normalized[:, 0, 0], normalized[:, 0, 1],
                            np.ones(len(normalized))))
    distortion_aware, _ = cv2.projectPoints(
        rays, np.zeros(3), np.zeros(3), DEPTH_K, DISTORTION_D)
    distortion_aware = distortion_aware.reshape(-1, 2)
    identity = pixels.reshape(-1, 2)
    return pinhole, distortion_aware, identity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("track_summary", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    summary = json.loads(args.track_summary.read_text(encoding="utf-8"))
    per_event = []
    all_pixels = []
    for event in summary["events"]:
        matches = event.get("matches", [])
        if not matches:
            continue
        pixels = [point for match in matches for point in (match["rgb0"], match["rgb1"])]
        pinhole, corrected, identity = map_pixels(pixels)
        n = len(pixels)
        pinhole_to_corrected = np.linalg.norm(corrected - pinhole, axis=1)
        rgb_depth_index_delta = np.linalg.norm(pinhole - identity, axis=1)
        item = {
            "target_index": event["target_index"],
            "max_cause": event["max_cause"],
            "valid_tracked_points": len(matches),
            "rgb_to_depth_index_delta_k_only": percentiles(rgb_depth_index_delta),
            "distortion_correction_delta_from_k_only": percentiles(pinhole_to_corrected),
            "distortion_corrected_depth_pixel_inside_640x400": int(np.count_nonzero(
                (corrected[:, 0] >= 0) & (corrected[:, 0] < 640) &
                (corrected[:, 1] >= 0) & (corrected[:, 1] < 400))),
        }
        per_event.append(item)
        all_pixels.extend(zip(rgb_depth_index_delta.tolist(),
                              pinhole_to_corrected.tolist()))
    result = {"track_summary": str(args.track_summary),
              "color_k": COLOR_K.tolist(), "depth_k": DEPTH_K.tolist(),
              "rgb_distortion_d": DISTORTION_D.tolist(),
              "pixel_mapping": "RGB distorted pixel -> undistort(K_color,D) -> project(K_depth,D)",
              "event_count_with_matches": len(per_event), "events": per_event}
    if all_pixels:
        values = np.asarray(all_pixels)
        result["all_tracked_pixel_statistics"] = {
            "feature_endpoint_count": len(values),
            "rgb_depth_index_delta_k_only": percentiles(values[:, 0]),
            "distortion_correction_delta_from_k_only": percentiles(values[:, 1]),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result["all_tracked_pixel_statistics"], indent=2))
    print("保存先:", args.output)


if __name__ == "__main__":
    main()
