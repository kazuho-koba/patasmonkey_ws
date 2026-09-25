#!/usr/bin/env python3
"""7月bagのWit/OAK IMUと再計算TFの姿勢を同じ時刻で照合する。

forensic frame eventはdepth撮像stampごとのbase姿勢を持つ。このstampに近いIMUだけを
MCAPから読み、Wit quaternionとTF roll/pitchの差、OAK IMUのorientation有効性・加速度
ノルムを集計する。Witは現在のreplay localizerの直接入力なので、この比較は独立な
姿勢正解検証ではなく、IMUからTFまでの伝搬・時刻整合確認として扱う。
"""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


TOPICS = {"/wit/imu": "wit", "/oak/imu/data": "oak"}


def stamp_ns(message):
    """ROS header stampを整数nanosecondへ変換する。"""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def rpy_from_quaternion(q):
    """IMU quaternionからroll/pitch/yawをradで取り出す。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees([roll, pitch, yaw])


def read_events(path):
    """frame_eventsをstamp単位にまとめる。同一frameの複数cell行は姿勢が同じ。"""
    by_stamp = {}
    with Path(path).open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            stamp = int(row["source_stamp_ns"])
            if stamp not in by_stamp:
                by_stamp[stamp] = {
                    "stamp_ns": stamp,
                    "tf_roll_deg": math.degrees(float(row["base_roll_rad"])),
                    "tf_pitch_deg": math.degrees(float(row["base_pitch_rad"])),
                    "tf_yaw_deg": math.degrees(float(row["base_yaw_rad"])),
                    "base_z_m": float(row["base_z_m"]),
                }
    return [by_stamp[key] for key in sorted(by_stamp)]


def read_imus(bag_dir, low_ns, high_ns):
    """必要区間のWit/OAK IMUだけを全MCAP分割からdeserializeする。"""
    messages = {name: [] for name in TOPICS.values()}
    rclpy.init()
    try:
        for path in sorted(Path(bag_dir).glob("*.mcap")):
            with path.open("rb") as stream:
                reader = make_reader(stream)
                for _, channel, record in reader.iter_messages(topics=list(TOPICS)):
                    topic = channel.topic if hasattr(channel, "topic") else channel
                    message = deserialize_message(record.data, Imu)
                    stamp = stamp_ns(message)
                    if low_ns <= stamp <= high_ns:
                        messages[TOPICS[topic]].append((stamp, message))
    finally:
        rclpy.shutdown()
    for name in messages:
        messages[name].sort(key=lambda item: item[0])
    return messages


def nearest(entries, stamps, target_ns):
    """指定stampにもっとも近いIMU sampleと符号付き時刻差を返す。"""
    index = bisect.bisect_left(stamps, target_ns)
    candidates = [i for i in (index - 1, index) if 0 <= i < len(stamps)]
    if not candidates:
        return None
    chosen = min(candidates, key=lambda i: abs(stamps[i] - target_ns))
    return entries[chosen], (entries[chosen][0] - target_ns) / 1e6


def interpolated_rpy(entries, stamps, target_ns):
    """TF stampへ挟み込みIMU quaternionのRPYを時間補間する。"""
    index = bisect.bisect_left(stamps, target_ns)
    if index == 0 or index == len(stamps):
        return None
    stamp0, message0 = entries[index - 1]
    stamp1, message1 = entries[index]
    if stamp1 == stamp0:
        return rpy_from_quaternion(message0.orientation)
    ratio = (target_ns - stamp0) / float(stamp1 - stamp0)
    first = rpy_from_quaternion(message0.orientation)
    second = rpy_from_quaternion(message1.orientation)
    # yawの±180度境界をまたいでも短い側を通るよう差分をwrapする。
    delta = (second - first + 180.0) % 360.0 - 180.0
    return first + ratio * delta


def summarize(events, messages, match_limit_ms):
    """TF stamp마다 가장 가까운 두 IMU를 비교하고 robust 통계를 계산한다."""
    result = {"matched": [], "topic_summary": {}}
    for name, entries in messages.items():
        stamps = [item[0] for item in entries]
        rows = []
        for event in events:
            match = nearest(entries, stamps, event["stamp_ns"])
            if match is None:
                continue
            (imu_stamp, message), offset_ms = match
            if abs(offset_ms) > match_limit_ms:
                continue
            quat = message.orientation
            qnorm = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
            orientation_valid = bool(message.orientation_covariance[0] >= 0 and
                                     0.9 <= qnorm <= 1.1)
            row = {
                "frame_stamp_ns": event["stamp_ns"], "imu_stamp_ns": imu_stamp,
                "time_offset_ms": offset_ms, "orientation_valid": orientation_valid,
                "orientation_covariance_0": message.orientation_covariance[0],
                "quaternion_norm": qnorm,
                "accel_xyz_mps2": [message.linear_acceleration.x,
                                   message.linear_acceleration.y,
                                   message.linear_acceleration.z],
                "accel_norm_mps2": math.sqrt(message.linear_acceleration.x**2 +
                                               message.linear_acceleration.y**2 +
                                               message.linear_acceleration.z**2),
                "gyro_xyz_rps": [message.angular_velocity.x,
                                 message.angular_velocity.y,
                                 message.angular_velocity.z],
            }
            if name == "wit" and orientation_valid:
                roll, pitch, yaw = rpy_from_quaternion(quat)
                interpolated = interpolated_rpy(entries, stamps, event["stamp_ns"])
                row.update({
                    "imu_roll_deg": float(roll), "imu_pitch_deg": float(pitch),
                    "imu_yaw_deg": float(yaw),
                    "tf_minus_imu_roll_deg": event["tf_roll_deg"] - float(roll),
                    "tf_minus_imu_pitch_deg": event["tf_pitch_deg"] - float(pitch),
                })
                if interpolated is not None:
                    row["tf_minus_interpolated_imu_roll_deg"] = (
                        event["tf_roll_deg"] - float(interpolated[0]))
                    row["tf_minus_interpolated_imu_pitch_deg"] = (
                        event["tf_pitch_deg"] - float(interpolated[1]))
            rows.append(row)
        result["matched"].extend({"topic": name, **row} for row in rows)
        summary = {"candidate_events": len(events), "matched_events": len(rows),
                   "imu_message_count_in_window": len(entries)}
        if rows:
            summary["nearest_offset_ms_p50_p95_abs"] = [
                float(np.percentile(np.abs([r["time_offset_ms"] for r in rows]), 50)),
                float(np.percentile(np.abs([r["time_offset_ms"] for r in rows]), 95)),
            ]
            summary["orientation_valid_count"] = sum(r["orientation_valid"] for r in rows)
            summary["accel_norm_mps2_p10_p50_p90"] = [
                float(np.percentile([r["accel_norm_mps2"] for r in rows], q))
                for q in (10, 50, 90)
            ]
            if name == "wit":
                for key in ("tf_minus_imu_roll_deg", "tf_minus_imu_pitch_deg",
                            "tf_minus_interpolated_imu_roll_deg",
                            "tf_minus_interpolated_imu_pitch_deg"):
                    values = [r[key] for r in rows if key in r]
                    summary[key + "_p50_p95_abs"] = [
                        float(np.median(values)), float(np.percentile(np.abs(values), 95))
                    ]
        result["topic_summary"][name] = summary
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("frame_events_csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--match-limit-ms", type=float, default=20.0)
    args = parser.parse_args()
    events = read_events(args.frame_events_csv)
    if not events:
        raise SystemExit("frame_eventsに対象行がありません")
    pad_ns = int(args.match_limit_ms * 1e6)
    messages = read_imus(args.bag, events[0]["stamp_ns"] - pad_ns,
                          events[-1]["stamp_ns"] + pad_ns)
    output = {"bag": str(args.bag), "event_count_unique_stamps": len(events),
              "match_limit_ms": args.match_limit_ms,
              **summarize(events, messages, args.match_limit_ms)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(json.dumps(output["topic_summary"], indent=2))
    print("保存先:", args.output)


if __name__ == "__main__":
    main()
