#!/usr/bin/env python3
"""Replay selected odometry inputs through robot_localization and retain results.

This is intentionally a process-level replay: it runs the same ``ekf_node``
and YAML used on the vehicle, records its output to MCAP, and exports durable
CSV/PNG/JSON artifacts.  It is suitable for local or global EKF experiments;
the caller supplies the YAML and the input topics.
"""

import argparse
import csv
import glob
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from typing import Dict, List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mcap.reader import make_reader
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


# /tf_static is required because robot_localization must transform the IMU
# frame into base_link before accepting its roll/pitch and angular-rate data.
DEFAULT_INPUT_TOPICS = [
    "/wheel/odometry", "/wit/imu", "/vio/odometry", "/tf_static",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Input rosbag2 MCAP directory")
    parser.add_argument("--config", required=True, type=Path,
                        help="robot_localization YAML used for this replay")
    parser.add_argument("--output-dir", required=True, type=Path,
                        help="New directory for all replay artifacts")
    parser.add_argument("--label", default="local_ekf",
                        help="Prefix for the re-estimated odometry topic")
    parser.add_argument("--node-name", default="ekf_local_node",
                        help="EKF node name matching the root key in the YAML")
    parser.add_argument("--rate", type=float, default=1.0,
                        help="rosbag playback rate; 1.0 is the fidelity default")
    parser.add_argument("--storage", default="mcap",
                        help="Input rosbag2 storage plugin (default: mcap)")
    parser.add_argument("--input-topic", action="append", dest="input_topics",
                        help="Input topic to replay (repeatable); defaults to wheel/Wit/VIO")
    parser.add_argument("--reference-topic", default="/odometry/local",
                        help="Existing odometry topic to export for comparison")
    parser.add_argument("--ros-domain-id", type=int, default=73,
                        help="Isolated ROS domain used only for the replay")
    parser.add_argument("--debug-ekf", action="store_true",
                        help="Save robot_localization internal debug trace")
    parser.add_argument("--vio-vertical-gate", action="store_true",
                        help="Run the production VIO safety gate during replay")
    return parser.parse_args()


def terminate(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def load_odometry(bag_dir: Path, topic: str) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    for mcap_file in sorted(glob.glob(str(bag_dir / "*.mcap"))):
        with open(mcap_file, "rb") as stream:
            for _, channel, record in make_reader(stream).iter_messages():
                if channel.topic != topic:
                    continue
                msg = deserialize_message(record.data, Odometry)
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                rows.append({
                    "t": stamp,
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "z": msg.pose.pose.position.z,
                    "qx": msg.pose.pose.orientation.x,
                    "qy": msg.pose.pose.orientation.y,
                    "qz": msg.pose.pose.orientation.z,
                    "qw": msg.pose.pose.orientation.w,
                    "vx": msg.twist.twist.linear.x,
                    "vy": msg.twist.twist.linear.y,
                    "vz": msg.twist.twist.linear.z,
                })
    return rows


def write_csv(path: Path, rows: List[Dict[str, float]]) -> None:
    fields = ["t", "t_rel", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz"]
    start = rows[0]["t"] if rows else 0.0
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({**row, "t_rel": row["t"] - start})


def plot_results(path: Path, estimate: List[Dict[str, float]],
                 vio: List[Dict[str, float]], reference: List[Dict[str, float]]) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(13, 5))
    for rows, name, color in ((estimate, "replayed EKF", "tab:blue"),
                              (reference, "recorded EKF", "tab:gray")):
        if rows:
            axes[0].plot([row["x"] for row in rows], [row["y"] for row in rows],
                         label=name, color=color, linewidth=1.2)
    axes[0].set_title("Horizontal trajectory")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend()

    for rows, name, color in ((estimate, "replayed EKF z", "tab:blue"),
                              (vio, "VIO z input", "tab:orange"),
                              (reference, "recorded EKF z", "tab:gray")):
        if rows:
            origin = rows[0]["t"]
            axes[1].plot([row["t"] - origin for row in rows], [row["z"] for row in rows],
                         label=name, color=color, linewidth=1.0)
    axes[1].set_title("Height observation and EKF state")
    axes[1].set_xlabel("time from each series start [s]")
    axes[1].set_ylabel("z [m]")
    axes[1].grid(True)
    axes[1].legend()
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def series_summary(rows: List[Dict[str, float]]) -> Dict[str, float]:
    if not rows:
        return {"count": 0}
    z_values = [row["z"] for row in rows]
    return {
        "count": len(rows),
        "duration_sec": rows[-1]["t"] - rows[0]["t"],
        "z_range_m": max(z_values) - min(z_values),
        "z_final_m": z_values[-1],
    }


def main() -> None:
    args = parse_args()
    input_topics = args.input_topics or DEFAULT_INPUT_TOPICS
    if args.output_dir.exists():
        raise SystemExit("Refusing to overwrite output directory: {}".format(args.output_dir))
    if not args.bag.is_dir() or not args.config.is_file():
        raise SystemExit("Input bag directory or EKF YAML does not exist")
    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")

    args.output_dir.mkdir(parents=True)
    recorded_bag = args.output_dir / "replayed_ekf_bag"
    output_topic = "/odometry/{}_replay".format(args.label)
    shutil.copy2(args.config, args.output_dir / "ekf_config.yaml")
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)

    logs = {}
    processes: List[subprocess.Popen] = []
    try:
        if args.vio_vertical_gate:
            gate_log = (args.output_dir / "vio_vertical_gate.log").open(
                "w", encoding="utf-8"
            )
            logs["vio_vertical_gate"] = gate_log
            gate = subprocess.Popen(
                ["ros2", "run", "pm_localization", "vio_vertical_gate_node"],
                env=env, stdout=gate_log, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            processes.append(gate)
            time.sleep(1.0)

        ekf_log = (args.output_dir / "ekf.log").open("w", encoding="utf-8")
        logs["ekf"] = ekf_log
        ekf_command = [
            "ros2", "run", "robot_localization", "ekf_node", "--ros-args",
            "--params-file", str(args.config),
            "-r", "__node:={}".format(args.node_name),
            "-r", "odometry/filtered:={}".format(output_topic),
            "-p", "use_sim_time:=true",
        ]
        if args.debug_ekf:
            ekf_command += [
                "-p", "debug:=true",
                "-p", "debug_out_file:={}".format(args.output_dir / "ekf_debug.log"),
            ]
        ekf = subprocess.Popen(
            ekf_command,
            env=env, stdout=ekf_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
        )
        processes.append(ekf)
        time.sleep(2.0)

        record_log = (args.output_dir / "record.log").open("w", encoding="utf-8")
        logs["record"] = record_log
        record_topics = [output_topic]
        if args.vio_vertical_gate:
            record_topics.append("/vio/vertical_gate/diagnostics")
        record = subprocess.Popen(
            ["ros2", "bag", "record", "--storage", "mcap", "-o", str(recorded_bag), *record_topics],
            env=env, stdout=record_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
        )
        processes.append(record)
        time.sleep(2.0)

        play_log = (args.output_dir / "play.log").open("w", encoding="utf-8")
        logs["play"] = play_log
        play = subprocess.Popen(
            ["ros2", "run", "pm_evaluation", "bag_clock_player", str(args.bag),
             "--rate", str(args.rate), *sum((["--topic", topic] for topic in input_topics), [])],
            env=env, stdout=play_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
        )
        processes.append(play)
        if play.wait() != 0:
            raise RuntimeError("ros2 bag play failed; see play.log")
        processes.remove(play)
        time.sleep(2.0)
        terminate(record)
        processes.remove(record)
        terminate(ekf)
        processes.remove(ekf)
    finally:
        for process in reversed(processes):
            terminate(process)
        for stream in logs.values():
            stream.close()

    estimate = load_odometry(recorded_bag, output_topic)
    if not estimate:
        raise RuntimeError(
            "EKF replay produced no odometry; inspect ekf.log, record.log, and play.log"
        )
    vio = load_odometry(args.bag, "/vio/odometry")
    reference = load_odometry(args.bag, args.reference_topic)
    write_csv(args.output_dir / "replayed_ekf.csv", estimate)
    write_csv(args.output_dir / "vio_input.csv", vio)
    write_csv(args.output_dir / "recorded_ekf.csv", reference)
    plot_results(args.output_dir / "ekf_replay_comparison.png", estimate, vio, reference)
    summary = {
        "input_bag": str(args.bag),
        "ekf_config": str(args.config),
        "output_topic": output_topic,
        "ekf_node_name": args.node_name,
        "input_topics": input_topics,
        "playback_rate": args.rate,
        "input_storage": args.storage,
        "vio_vertical_gate": args.vio_vertical_gate,
        "replayed_ekf": series_summary(estimate),
        "vio_input": series_summary(vio),
        "recorded_ekf": series_summary(reference),
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
