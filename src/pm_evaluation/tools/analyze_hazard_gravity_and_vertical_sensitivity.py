#!/usr/bin/env python3
"""OAK加速度の重力整合とhazard追跡点の縦方向幾何感度を診断する。

OAK IMU orientationが無効なbagでも、linear_accelerationをKalibr IMU-camera姿勢で
cam0/base軸へ回し、時刻の近いreplay TFでodomへ回したとき重力方向と整合するかを見る。
同時に、黒地点の追跡特徴についてcamera mount z/pitch、depth、pixel vの摂動が
2フレーム間のodom高さ差へ与える量を再投影して測る。

どちらもground truth測量ではない。OAK個体のbag内identity欠落、URDF mount仮定、
IMU中の走行加速度が残るため、独立証明でなく仮定つきの感度・整合性検査である。
"""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml


G = 9.80665
DEPTH_K = (574.28826904, 574.28826904, 354.75085449, 215.23262024)


def rpy_matrix(roll, pitch, yaw):
    """roll/pitch/yawからZYX順の3x3回転行列を作る。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]], dtype=np.float64)


def quat_matrix(q):
    """[x,y,z,w] quaternionを正規化して回転行列へ変換する。"""
    x, y, z, w = np.asarray(q, dtype=np.float64)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def read_frame_events(path):
    """frame_eventsをdepth stampごとに一意化し、base/camera姿勢を行列化する。"""
    unique = {}
    with Path(path).open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            stamp = int(row["source_stamp_ns"])
            if stamp in unique:
                continue
            base_rpy = [float(row[key]) for key in
                        ("base_roll_rad", "base_pitch_rad", "base_yaw_rad")]
            camera_q = [float(row[key]) for key in
                        ("camera_qx", "camera_qy", "camera_qz", "camera_qw")]
            unique[stamp] = {
                "stamp_ns": stamp, "base_rpy": base_rpy,
                "base_R": rpy_matrix(*base_rpy),
                "base_t": np.array([float(row[k]) for k in
                                     ("base_x_m", "base_y_m", "base_z_m")]),
                "camera_R": quat_matrix(camera_q),
                "camera_t": np.array([float(row[k]) for k in
                                      ("camera_x_m", "camera_y_m", "camera_z_m")]),
            }
    return [unique[key] for key in sorted(unique)]


def decode_mcap(bag_dir, low_ns, high_ns):
    """指定時間帯のOAK IMUだけMCAPから読む。ROS依存はこの診断スクリプトだけ。"""
    import rclpy
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    samples = []
    rclpy.init()
    try:
        for path in sorted(Path(bag_dir).glob("*.mcap")):
            with path.open("rb") as stream:
                reader = make_reader(stream)
                for _, channel, record in reader.iter_messages(topics=["/oak/imu/data"]):
                    msg = deserialize_message(record.data, Imu)
                    stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                    if low_ns <= stamp <= high_ns:
                        samples.append({
                            "stamp_ns": stamp, "frame_id": msg.header.frame_id,
                            "accel": np.array([msg.linear_acceleration.x,
                                                msg.linear_acceleration.y,
                                                msg.linear_acceleration.z]),
                            "gyro": np.array([msg.angular_velocity.x,
                                               msg.angular_velocity.y,
                                               msg.angular_velocity.z]),
                            "orientation_cov0": float(msg.orientation_covariance[0]),
                        })
    finally:
        rclpy.shutdown()
    return sorted(samples, key=lambda item: item["stamp_ns"])


def nearest_sample(samples, stamps, target_ns, limit_ms=10.0):
    """depth stampに最近傍OAK IMU sampleを割り当てる。"""
    index = bisect.bisect_left(stamps, target_ns)
    choices = [i for i in (index - 1, index) if 0 <= i < len(stamps)]
    if not choices:
        return None
    i = min(choices, key=lambda j: abs(stamps[j] - target_ns))
    delta_ms = (stamps[i] - target_ns) / 1e6
    return (samples[i], delta_ms) if abs(delta_ms) <= limit_ms else None


def percentile_summary(values):
    """数値のp50/p90と件数を返す。"""
    if not values:
        return {"count": 0}
    values = np.asarray(values, dtype=np.float64)
    return {"count": len(values), "p50": float(np.percentile(values, 50)),
            "p90": float(np.percentile(values, 90))}


def analyze_gravity(events, samples, calibration_path, acceleration_gate, gyro_gate,
                    hazard_stamps):
    """Kalibr IMU-camera rotationで加速度をodomへ移し、重力方向を比較する。"""
    calibration_text = Path(calibration_path).read_text(encoding="utf-8")
    calibration = yaml.safe_load("\n".join(calibration_text.splitlines()[1:]))
    R_imu_cam0 = np.asarray(calibration["cam0"]["T_imu_cam"], dtype=np.float64)[:3, :3]
    # URDFはcam0 opticalをbase_linkへrpy(-90°,0,-90°)で固定している。
    R_base_cam0 = rpy_matrix(-math.pi / 2, 0.0, -math.pi / 2)
    # T_imu_camはcam0 vectorをIMU座標へ回す。imu加速度をbaseへ戻す回転。
    R_base_imu = (R_imu_cam0 @ R_base_cam0.T).T
    stamps = [sample["stamp_ns"] for sample in samples]
    matches = []
    for event in events:
        match = nearest_sample(samples, stamps, event["stamp_ns"])
        if match is None:
            continue
        sample, offset_ms = match
        accel_base = R_base_imu @ sample["accel"]
        accel_odom = event["base_R"] @ accel_base
        norm = float(np.linalg.norm(accel_odom))
        gyro_norm = float(np.linalg.norm(sample["gyro"]))
        # REP-145の加速度符号に依存せず、測定ベクトルと鉛直軸の鋭角を使う。
        horizontal = float(np.linalg.norm(accel_odom[:2]))
        tilt_error_deg = math.degrees(math.atan2(horizontal, abs(float(accel_odom[2]))))
        speed = None
        matches.append({"stamp_ns": event["stamp_ns"], "time_offset_ms": offset_ms,
                        "accel_odom": accel_odom, "accel_norm": norm,
                        "accel_imu": sample["accel"], "gyro_imu": sample["gyro"],
                        "gyro_norm": gyro_norm, "tilt_error_deg": tilt_error_deg,
                        "frame_id": sample["frame_id"], "base_xy": event["base_t"][:2],
                        "speed": speed})

    # 対応frame間のodom水平速度を中心差分で概算し、停止・低速窓を区別する。
    for index, item in enumerate(matches):
        left, right = max(0, index - 1), min(len(matches) - 1, index + 1)
        dt = (matches[right]["stamp_ns"] - matches[left]["stamp_ns"]) / 1e9
        if dt > 0:
            item["speed"] = float(np.linalg.norm(
                matches[right]["base_xy"] - matches[left]["base_xy"]) / dt)
    # まず加速度normとgyroで静穏候補を選び、さらに速度でvehicle動作の影響を落とす。
    quiet = [item for item in matches
             if abs(item["accel_norm"] - G) <= acceleration_gate
             and item["gyro_norm"] <= gyro_gate and item["speed"] <= 0.15]
    hazard_frames = []
    for item in matches:
        if item["stamp_ns"] in hazard_stamps:
            hazard_frames.append({key: (value.tolist() if isinstance(value, np.ndarray)
                                        else value)
                                  for key, value in item.items()})
    return {
        "oak_imu_match_count": len(matches),
        "imu_frame_ids": sorted(set(item["frame_id"] for item in matches)),
        "orientation_covariance_invalid_count": sum(
            1 for sample in samples if sample["orientation_cov0"] < 0),
        "imu_sample_count_in_window": len(samples),
        "kalibr_serial": "19443010C10EF91200",
        "assumed_camera_to_base_rotation_source": "URDF cam0 optical固定姿勢",
        "mount_pose_assumption": "IMUからbaseへの回転はKalibr cam0-IMU外部姿勢+URDF cam0/baseで構成。bag内serialは未記録。",
        "all_matched": {
            "accel_norm_mps2": percentile_summary([x["accel_norm"] for x in matches]),
            "gyro_norm_rps": percentile_summary([x["gyro_norm"] for x in matches]),
            "gravity_axis_misalignment_deg": percentile_summary(
                [x["tilt_error_deg"] for x in matches]),
            "speed_mps": percentile_summary([x["speed"] for x in matches]),
        },
        "quiet_gate": {"abs_accel_norm_minus_g_max_mps2": acceleration_gate,
                       "gyro_norm_max_rps": gyro_gate, "speed_max_mps": 0.15},
        "quiet_subset_count": len(quiet),
        "quiet_subset": {
            "gravity_axis_misalignment_deg": percentile_summary(
                [x["tilt_error_deg"] for x in quiet]),
            "accel_odom_mps2_median_xyz": (np.median(
                np.asarray([x["accel_odom"] for x in quiet]), axis=0).tolist()
                if quiet else None),
            "speed_mps": percentile_summary([x["speed"] for x in quiet]),
            "accel_norm_mps2": percentile_summary([x["accel_norm"] for x in quiet]),
            "gyro_norm_rps": percentile_summary([x["gyro_norm"] for x in quiet]),
        },
        "selected_hazard_source_frames": hazard_frames,
        "warning": "gravity direction is only a weak check: acceleration may include vehicle dynamics; no independent tilt ground truth.",
    }


def world_point(depth_pixel, depth_m, camera_R, camera_t, depth_k=DEPTH_K):
    """depth pixelとaxial rangeをcamera opticalからodom XYZへback-projectする。"""
    fx, fy, cx, cy = depth_k
    u, v = depth_pixel
    point_cam = np.array([(u - cx) * depth_m / fx,
                          (v - cy) * depth_m / fy, depth_m])
    return camera_R @ point_cam + camera_t


def perturb_attitude(base_R, epsilon_rad, axis):
    """base座標のpitchまたはroll誤差を姿勢へ適用する。"""
    if axis == "pitch":
        correction = rpy_matrix(0.0, epsilon_rad, 0.0)
    else:
        correction = rpy_matrix(epsilon_rad, 0.0, 0.0)
    return base_R @ correction


def geometry_sensitivity(track_path, events_csv, height_offset_m=0.12):
    """主要黒地点の追跡特徴でheight/extrinsic/depth/v/attitude感度を測る。"""
    with Path(events_csv).open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    pose_by_stamp = {}
    for row in rows:
        stamp = int(row["source_stamp_ns"])
        if stamp in pose_by_stamp:
            continue
        rpy = [float(row[key]) for key in
               ("base_roll_rad", "base_pitch_rad", "base_yaw_rad")]
        pose_by_stamp[stamp] = {
            "R_base": rpy_matrix(*rpy),
            "t_base": np.array([float(row[k]) for k in
                                 ("base_x_m", "base_y_m", "base_z_m")]),
            "R_cam": quat_matrix([float(row[k]) for k in
                                   ("camera_qx", "camera_qy", "camera_qz", "camera_qw")]),
            "t_cam": np.array([float(row[k]) for k in
                                ("camera_x_m", "camera_y_m", "camera_z_m")]),
        }
    track = json.loads(Path(track_path).read_text(encoding="utf-8"))
    results = []
    for event in track["events"]:
        if event["target_index"] not in (5, 12, 13) or not event.get("matches"):
            continue
        p0, p1 = pose_by_stamp[event["source_stamp0_ns"]], pose_by_stamp[event["source_stamp1_ns"]]
        pose = [p0, p1]
        # カメラ固定mountをbase座標へ戻す。frame stamp間で一定値として扱う。
        extrinsic_R = [x["R_base"].T @ x["R_cam"] for x in pose]
        extrinsic_t = [x["R_base"].T @ (x["t_cam"] - x["t_base"]) for x in pose]
        point_deltas = []
        effects = {key: [] for key in ("mount_z_plus_12cm", "mount_pitch_plus_1deg",
                                       "mount_roll_plus_1deg", "depth_plus_5cm_both",
                                       "depth_scale_plus_1pct_both", "pixel_v_plus_1px_both",
                                       "pixel_v_plus_1px_frame1_only",
                                       "frame1_pitch_plus_0p1deg",
                                       "frame1_roll_plus_0p1deg")}
        required_depth_bias_m = []
        required_frame1_pitch_deg = []
        for match in event["matches"]:
            xyz = []
            rays = []
            for i, frame_key in enumerate(("0", "1")):
                dpx = np.asarray(match["depth_pixel" + frame_key], dtype=np.float64)
                depth = float(match["depth_m" + frame_key])
                xyz.append(world_point(dpx, depth, pose[i]["R_cam"], pose[i]["t_cam"]))
                rays.append(np.array([(dpx[0] - DEPTH_K[2]) / DEPTH_K[0],
                                      (dpx[1] - DEPTH_K[3]) / DEPTH_K[1], 1.0]))
            baseline_delta = float(xyz[1][2] - xyz[0][2])
            point_deltas.append(baseline_delta)

            def changed_world(i, depth_delta=0.0, depth_scale=1.0, v_delta=0.0,
                              mount_z=0.0, mount_pitch=0.0, mount_roll=0.0,
                              pose_pitch=0.0, pose_roll=0.0):
                # 静的カメラ外部姿勢の摂動は各frameのbase poseとともにodomへ回す。
                Rb = pose[i]["R_base"] @ rpy_matrix(pose_roll, pose_pitch, 0.0)
                correction = rpy_matrix(mount_roll, mount_pitch, 0.0)
                Rbc = correction @ extrinsic_R[i]
                tc = pose[i]["t_base"] + Rb @ (
                    extrinsic_t[i] + np.array([0.0, 0.0, mount_z]))
                Rc = Rb @ Rbc
                px = np.asarray(match["depth_pixel" + str(i)], dtype=np.float64).copy()
                px[1] += v_delta
                dep = float(match["depth_m" + str(i)]) * depth_scale + depth_delta
                return world_point(px, dep, Rc, tc)

            test_cases = {
                "mount_z_plus_12cm": [(0, {"mount_z": height_offset_m}),
                                      (1, {"mount_z": height_offset_m})],
                "mount_pitch_plus_1deg": [(0, {"mount_pitch": math.radians(1)}),
                                          (1, {"mount_pitch": math.radians(1)})],
                "mount_roll_plus_1deg": [(0, {"mount_roll": math.radians(1)}),
                                         (1, {"mount_roll": math.radians(1)})],
                "depth_plus_5cm_both": [(0, {"depth_delta": 0.05}),
                                        (1, {"depth_delta": 0.05})],
                "depth_scale_plus_1pct_both": [(0, {"depth_scale": 1.01}),
                                                (1, {"depth_scale": 1.01})],
                "pixel_v_plus_1px_both": [(0, {"v_delta": 1}), (1, {"v_delta": 1})],
                "pixel_v_plus_1px_frame1_only": [(0, {}), (1, {"v_delta": 1})],
                "frame1_pitch_plus_0p1deg": [(0, {}), (1, {"mount_pitch": math.radians(0.1)})],
                "frame1_roll_plus_0p1deg": [(0, {}), (1, {"pose_roll": math.radians(0.1)})],
            }
            test_cases["frame1_pitch_plus_0p1deg"] = [
                (0, {}), (1, {"pose_pitch": math.radians(0.1)})]
            for key, pair in test_cases.items():
                changed = [changed_world(i, **kwargs) for i, kwargs in pair]
                effects[key].append(float(changed[1][2] - changed[0][2] - baseline_delta))

            # 2フレーム間の相対depth biasが単独でbaseline Δzを消すために必要な量。
            dz_dd = float((pose[1]["R_cam"] @ rays[1])[2])
            if abs(dz_dd) > 1e-4:
                required_depth_bias_m.append(-baseline_delta / dz_dd)
            # frame1のpitch-only姿勢誤差でbaseline Δzを消す線形近似。
            epsilon = math.radians(0.01)
            changed = changed_world(1, pose_pitch=epsilon)
            derivative = (changed[2] - xyz[1][2]) / epsilon
            if abs(derivative) > 1e-4:
                required_frame1_pitch_deg.append(math.degrees(-baseline_delta / derivative))

        results.append({
            "target_index": event["target_index"], "max_cause": event["max_cause"],
            "source_interval_s": event["source_interval_s"],
            "support_relative_height_delta_m": (
                event["support_map_elevation1_m"] - event["support_map_elevation0_m"]),
            "tracked_point_count": len(point_deltas),
            "baseline_delta_z_m": percentile_summary(point_deltas),
            "static_mount_or_measurement_perturbation_effect_on_delta_z_m": {
                key: percentile_summary(values) for key, values in effects.items()},
            "per_feature_delta_depth_bias_to_cancel_delta_z_m": percentile_summary(
                np.abs(required_depth_bias_m).tolist()),
            "per_feature_frame1_pitch_error_to_cancel_delta_z_deg": percentile_summary(
                np.abs(required_frame1_pitch_deg).tolist()),
            "interpretation": "constant mount/depth offsets are common-mode candidates; their effect on inter-frame delta differs from their absolute elevation bias.",
        })
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("frame_events_csv", type=Path)
    parser.add_argument("track_summary", type=Path)
    parser.add_argument("imu_camera_yaml", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--accel-gate", type=float, default=0.30)
    parser.add_argument("--gyro-gate", type=float, default=0.15)
    args = parser.parse_args()
    events = read_frame_events(args.frame_events_csv)
    if not events:
        raise SystemExit("frame_eventsが空です")
    samples = decode_mcap(args.bag, events[0]["stamp_ns"] - 20_000_000,
                          events[-1]["stamp_ns"] + 20_000_000)
    tracks = json.loads(args.track_summary.read_text(encoding="utf-8"))
    hazard_stamps = {stamp for event in tracks["events"]
                     if event["target_index"] in (5, 12, 13)
                     for stamp in (event["source_stamp0_ns"], event["source_stamp1_ns"])}
    output = {
        "bag": str(args.bag), "unique_depth_pose_stamp_count": len(events),
        "gravity_direction_check": analyze_gravity(
            events, samples, args.imu_camera_yaml, args.accel_gate, args.gyro_gate,
            hazard_stamps),
        "vertical_geometry_sensitivity": geometry_sensitivity(
            args.track_summary, args.frame_events_csv),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(json.dumps({"gravity_direction_check": output["gravity_direction_check"],
                      "vertical_geometry_sensitivity": output["vertical_geometry_sensitivity"]},
                     indent=2))
    print("保存先:", args.output)


if __name__ == "__main__":
    main()
