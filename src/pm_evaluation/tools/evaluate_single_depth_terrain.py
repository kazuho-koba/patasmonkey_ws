#!/usr/bin/env python3
"""MCAPのdepth 1枚だけを全pixel投影し、現行terrain featureをオフライン評価する。

時間融合・bag再生・ROS購読は行わない。camera→baseはbagのstatic TFを用い、
base→診断座標は指定roll/pitchだけを適用する。最新odomの姿勢を自動再計算する
ツールではないため、姿勢未指定時の水平仮定と融合問題の確定を混同しない。
"""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
import yaml

from pm_perception.depth_projection import sampled_points, transform_points
from pm_perception.rolling_elevation_grid import RollingElevationGrid
from pm_perception.terrain_features import compute_terrain_features


def stamp_ns(message):
    """撮像時刻をns整数にする。MCAP記録時刻とは区別する。"""
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def messages(bag, topics):
    """必要topicだけをstreamし、画像列をメモリに保持しない。MCAP専用。"""
    files = sorted(Path(bag).glob("*.mcap"))
    if not files:
        raise ValueError("MCAPが見つかりません（このツールはSQLite未対応）")
    for path in files:
        with path.open("rb") as stream:
            yield from make_reader(stream).iter_messages(topics=topics)


def evaluate_points(points, params, stamp, center=(0.0, 0.0), heading_yaw=0.0,
                    relative_elevation_offset=0.0, grid=None, observation_variance=None):
    """空gridに1回だけ入力する。groundはセル最小zで、時間方向の融合はない。

    runtimeと同じ初期化・feature関数を再利用する。同一画像内の複数pixelの
    XYセル集約は残す。raw_obstacleはconfidence gate前の単一画像高さ幅。
    """
    # 通常は空grid。明示的な時間融合対照だけは呼び出し側所有のgridを再利用する。
    if grid is None:
        grid = RollingElevationGrid(params["map_size_x"], params["map_size_y"], params["resolution"])
    grid.recenter(*center)
    grid.fuse_points(
        points[:, 0], points[:, 1], points[:, 2], stamp,
        params["ground_merge_threshold"], params["obstacle_min_height"],
        observation_variance=(np.full(len(points), params["measurement_variance"], dtype=np.float32)
                              if observation_variance is None else observation_variance),
        relative_elevation_offset=relative_elevation_offset,
        observation_decay_time=params["observation_decay_time"],
    )
    layers = grid.stage2_layers(
        params["measurement_variance"], stamp,
        params["obstacle_confidence_min"], params["observation_decay_time"],
    )
    # min/max/countを新規に集計し、NPZに保存する。未知セルの高さはNaNのまま。
    ix = np.floor((points[:, 0] - grid.origin_x) / grid.resolution).astype(int)
    iy = np.floor((points[:, 1] - grid.origin_y) / grid.resolution).astype(int)
    inside = (ix >= 0) & (ix < grid.width) & (iy >= 0) & (iy < grid.height)
    slots = iy[inside] * grid.width + ix[inside]
    minimum = np.full(grid.cell_count, np.inf, dtype=np.float32)
    maximum = np.full(grid.cell_count, -np.inf, dtype=np.float32)
    np.minimum.at(minimum, slots, points[inside, 2])
    np.maximum.at(maximum, slots, points[inside, 2])
    count = np.bincount(slots, minlength=grid.cell_count).reshape(grid.height, grid.width)
    minimum = minimum.reshape(count.shape)
    maximum = maximum.reshape(count.shape)
    minimum[count == 0] = np.nan
    maximum[count == 0] = np.nan
    raw_obstacle = maximum - minimum
    raw_obstacle[raw_obstacle < params["obstacle_min_height"]] = np.nan

    def features(obstacle):
        return compute_terrain_features(
            layers["relative_elevation"], layers["age_seconds"], grid.resolution,
            params["feature_max_observation_age"], params["feature_neighborhood_radius_cells"],
            params["feature_min_neighbors"], params["hazard_slope_limit_deg"],
            params["hazard_roughness_limit"], params["hazard_step_limit"],
            obstacle_height=obstacle, obstacle_limit=params["hazard_obstacle_height_limit"],
            heading_yaw=heading_yaw, step_min_side_neighbors=params["step_min_side_neighbors"],
            include_diagnostics=True,
        )

    current = features(layers["obstacle_height"])
    reference = features(raw_obstacle)
    arrays = dict(layers)
    arrays.update(current)
    arrays.update(cell_min=minimum, cell_max=maximum, pixel_count=count,
                  raw_obstacle=raw_obstacle, reference_hazard=reference["hazard"],
                  origin=np.array([grid.origin_x, grid.origin_y]), resolution=np.array(grid.resolution))
    return arrays


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--output", required=True)
    parser.add_argument("--seconds", type=float, default=30.0, help="最初のdepth stampからの秒")
    parser.add_argument("--config", required=True, help="比較baseline YAML")
    parser.add_argument("--override", help="閾値等の上書きYAML")
    parser.add_argument("--resolution", type=float, help="セル幅[m]。YAMLより優先")
    parser.add_argument("--stride", type=int, default=1, help="既定1＝間引きなし。対照用に4も指定可能")
    parser.add_argument("--single-frame-obstacle", action="store_true",
                        help="1枚のセル高さ幅からconfidence待ちなしでobstacleを評価し、参考hazardを主指標とする")
    parser.add_argument("--camera-frame", default="rgb_camera_optical_frame", help="7月bagのTF名")
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    args = parser.parse_args()
    if args.seconds < 0 or args.stride < 1:
        parser.error("secondsは0以上、strideは1以上")
    params = yaml.safe_load(Path(args.config).read_text())["depth_elevation_mapper"]["ros__parameters"]
    if args.override:
        params.update(yaml.safe_load(Path(args.override).read_text())["depth_elevation_mapper"]["ros__parameters"])
    if args.resolution is not None:
        params["resolution"] = args.resolution
    rclpy.init()
    try:
        # 選択は最初のdepth撮像時刻+seconds以降の最初の画像。選択時刻を必ず保存する。
        first = None
        depth = None
        for _, _, record in messages(args.bag, [params["depth_topic"]]):
            candidate = deserialize_message(record.data, Image)
            if first is None:
                first = stamp_ns(candidate)
            if stamp_ns(candidate) >= first + int(args.seconds * 1e9):
                depth = candidate
                break
        if depth is None:
            raise ValueError("指定時刻のdepthがありません")
        stamp = stamp_ns(depth)
        buffer = Buffer()
        info = color = None
        # static TFはbag全体から収集する。camera_info/RGBは撮像時刻が最も近いものを選ぶ。
        # 全depth画像の再走査はしない。RGBは比較用の参考画像でpixel位置合わせはしない。
        for _, channel, record in messages(args.bag, ["/tf_static", params["camera_info_topic"], "/oak/color/image_raw"]):
            if channel.topic == "/tf_static":
                for tf in deserialize_message(record.data, TFMessage).transforms:
                    buffer.set_transform_static(tf, "offline_bag")
            else:
                message = deserialize_message(record.data, CameraInfo if channel.topic == params["camera_info_topic"] else Image)
                previous = info if channel.topic == params["camera_info_topic"] else color
                if previous is None or abs(stamp_ns(message) - stamp) < abs(stamp_ns(previous) - stamp):
                    if channel.topic == params["camera_info_topic"]:
                        info = message
                    else:
                        color = message
        if info is not None:
            if info.width != depth.width or info.height != depth.height:
                raise ValueError("camera_infoとdepth解像度が不一致。黙ってintrinsicsを流用しません")
            intrinsics = [info.k[0], info.k[4], info.k[2], info.k[5]]
            intrinsic_source = "bag camera_info K"
        else:
            if not params.get("allow_fallback_intrinsics", False):
                raise ValueError("camera_infoがなくfallbackが許可されていません")
            if depth.width != params["fallback_width"] or depth.height != params["fallback_height"]:
                raise ValueError("fallbackとdepth解像度が不一致")
            intrinsics = [params["fallback_" + name] for name in ["fx", "fy", "cx", "cy"]]
            intrinsic_source = "YAML fallback"
        optical = sampled_points(depth, *intrinsics, args.stride, params["min_depth"], params["max_depth"])
        tf = buffer.lookup_transform("base_link", args.camera_frame, Time(nanoseconds=stamp))
        base_points = transform_points(optical, tf.transform)
        # R_y(pitch) R_x(roll)でbaseをgravity-aligned診断座標へ向ける。
        # x/y/z平行移動とyawは不要。roll/pitch=0は姿勢推定を検証する条件ではない。
        roll, pitch = np.radians([args.roll_deg, args.pitch_deg])
        rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        points = base_points @ (ry @ rx).T
        arrays = evaluate_points(points, params, stamp)
        arrays["selected_hazard"] = arrays["reference_hazard"] if args.single_frame_obstacle else arrays["hazard"]
        output = Path(args.output)
        output.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(output / "single_frame.npz", **arrays)
        np.save(output / "depth_mm.npy", np.frombuffer(depth.data, dtype=">u2" if depth.is_bigendian else "<u2").reshape(depth.height, depth.step // 2)[:, :depth.width])
        summary = dict(depth_stamp_ns=stamp, seconds_from_first=(stamp-first)/1e9,
                       depth_frame=depth.header.frame_id, camera_frame=args.camera_frame,
                       intrinsics=intrinsics, intrinsics_source=intrinsic_source,
                       stride=args.stride, valid_pixels=len(points), roll_deg=args.roll_deg,
                       pitch_deg=args.pitch_deg, pose_source="specified roll/pitch, not reconstructed odom",
                       parameters=params, temporal_fusion=False,
                       single_frame_obstacle=args.single_frame_obstacle,
                       primary_metric="reference_hazard" if args.single_frame_obstacle else "hazard")
        # 値100の丸めではなく、ここでは保存した生のhazard>=1を数える。
        for name in ["hazard", "reference_hazard"]:
            known = np.isfinite(arrays[name])
            black = known & (arrays[name] >= 1)
            summary[name] = dict(known=int(known.sum()), black=int(black.sum()),
                                 black_percent=float(100*black.sum()/known.sum()) if known.any() else None,
                                 unknown=int((~known).sum()))
        summary["obstacle_valid_cells_current"] = int(np.isfinite(arrays["obstacle_height"]).sum())
        summary["selected_hazard"] = summary[summary["primary_metric"]]
        summary["observed_cells"] = int((arrays["pixel_count"] > 0).sum())
        summary["color_stamp_ns"] = stamp_ns(color) if color is not None else None
        (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
        # 固定thresholdスケール。hazardは白0〜黒1、unknownは紫で区別する。
        fig, axes = plt.subplots(2, 4, figsize=(19, 9))
        extent = [arrays["origin"][0], arrays["origin"][0]+arrays["hazard"].shape[1]*params["resolution"],
                  arrays["origin"][1], arrays["origin"][1]+arrays["hazard"].shape[0]*params["resolution"]]
        panels = [("elevation", None), ("slope_deg", params["hazard_slope_limit_deg"]),
                  ("roughness", params["hazard_roughness_limit"]), ("step_height", params["hazard_step_limit"]),
                  ("raw_obstacle", params["hazard_obstacle_height_limit"]), ("hazard", 1), ("reference_hazard", 1)]
        for ax, (name, maximum) in zip(axes.flat, panels):
            cmap = plt.get_cmap("gray_r" if "hazard" in name else "viridis").copy()
            cmap.set_bad("orchid")
            image = ax.imshow(arrays[name], origin="lower", extent=extent, cmap=cmap,
                              vmin=0 if maximum else None, vmax=maximum)
            ax.set_title(name)
            ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
            fig.colorbar(image, ax=ax)
        if color is not None and color.encoding in ("rgb8", "bgr8"):
            pixels = np.frombuffer(color.data, dtype=np.uint8).reshape(color.height, color.step)[:, :color.width*3].reshape(color.height, color.width, 3)
            axes.flat[7].imshow(pixels if color.encoding == "rgb8" else pixels[:, :, ::-1])
            axes.flat[7].set_title("nearest RGB (not pixel-aligned)")
        fig.suptitle("Single depth, no temporal fusion; roll/pitch %.2f/%.2f deg" % (args.roll_deg, args.pitch_deg))
        fig.tight_layout()
        fig.savefig(output / "single_frame.png", dpi=140)
        plt.close(fig)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
