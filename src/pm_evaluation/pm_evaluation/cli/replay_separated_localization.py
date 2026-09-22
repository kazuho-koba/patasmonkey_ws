#!/usr/bin/env python3
"""Replay the separated off-road localizer and retain MCAP/CSV/PNG evidence."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pm_evaluation.cli.replay_ekf_bag import (
    DEFAULT_INPUT_TOPICS,
    load_odometry,
    plot_results,
    series_summary,
    terminate,
    write_csv,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--horizontal-config", required=True, type=Path)
    parser.add_argument("--heading-config", type=Path,
                        help="Optional heading_initializer YAML to test")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--ros-domain-id", type=int, default=84)
    return parser.parse_args()


def start(command, env, log_path, processes, logs):
    stream = log_path.open("w", encoding="utf-8")
    logs.append(stream)
    process = subprocess.Popen(
        command, env=env, stdout=stream, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    processes.append(process)
    return process


def safe_output_summary(rows):
    """Report horizontal extent explicitly; z-only summaries hide divergence."""
    result = series_summary(rows)
    if not rows:
        return result
    xs = [row["x"] for row in rows]
    ys = [row["y"] for row in rows]
    result.update({
        "x_range_m": max(xs) - min(xs),
        "y_range_m": max(ys) - min(ys),
        "horizontal_endpoint_m": (xs[-1] ** 2 + ys[-1] ** 2) ** 0.5,
    })
    return result


def plot_safe_output(path, composed, vertical):
    """Plot only the safe output, without historical divergent data scaling it."""
    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    if composed:
        xs = [row["x"] for row in composed]
        ys = [row["y"] for row in composed]
        axes[0].plot(xs, ys, color="tab:blue", linewidth=1.2,
                     label="separated local odometry")
        axes[0].scatter([xs[0]], [ys[0]], color="tab:green", label="start", zorder=3)
        axes[0].scatter([xs[-1]], [ys[-1]], color="tab:red", label="end", zorder=3)
    axes[0].set_title("Separated localizer: horizontal trajectory")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend()

    if vertical:
        origin = vertical[0]["t"]
        axes[1].plot([row["t"] - origin for row in vertical],
                     [row["z"] for row in vertical], color="tab:blue",
                     linewidth=1.0, label="validated VIO height / held output")
    axes[1].set_title("Attitude-height observer output")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("z [m]")
    axes[1].grid(True)
    axes[1].legend()
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main():
    args = parse_args()
    if args.output_dir.exists():
        raise SystemExit("Refusing to overwrite output directory: {}".format(args.output_dir))
    if args.rate <= 0 or not args.bag.is_dir():
        raise SystemExit("Bag must exist and --rate must be positive")
    args.output_dir.mkdir(parents=True)
    shutil.copy2(args.horizontal_config, args.output_dir / "horizontal_ekf_config.yaml")
    if args.heading_config:
        shutil.copy2(args.heading_config, args.output_dir / "heading_initializer.yaml")
    output_topic = "/odometry/local_separated_replay"
    horizontal_topic = "/odometry/local_horizontal_replay"
    vertical_topic = "/odometry/local_vertical_replay"
    recorded_bag = args.output_dir / "replayed_localization_bag"
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    processes, logs = [], []
    try:
        start(["ros2", "run", "pm_localization", "vio_vertical_gate_node"], env,
              args.output_dir / "vio_vertical_gate.log", processes, logs)
        start(["ros2", "run", "pm_localization", "vio_twist_gate_node"], env,
              args.output_dir / "vio_twist_gate.log", processes, logs)
        time.sleep(1)
        start([
            "ros2", "run", "robot_localization", "ekf_node", "--ros-args",
            "--params-file", str(args.horizontal_config),
            "-r", "__node:=ekf_local_horizontal_node",
            "-r", "odometry/filtered:={}".format(horizontal_topic),
            "-p", "use_sim_time:=true",
        ], env, args.output_dir / "horizontal_ekf.log", processes, logs)
        if args.heading_config:
            start([
                "ros2", "run", "pm_localization", "heading_initializer_node",
                "--ros-args", "--params-file", str(args.heading_config),
            ], env, args.output_dir / "heading_initializer.log", processes, logs)
        start([
            "ros2", "run", "pm_localization", "attitude_height_observer_node",
            "--ros-args",
            "-p", "output_topic:={}".format(vertical_topic),
            "-p", "use_sim_time:=true",
        ], env, args.output_dir / "attitude_height_observer.log", processes, logs)
        start([
            "ros2", "run", "pm_localization", "local_odometry_composer_node",
            "--ros-args",
            "-p", "horizontal_topic:={}".format(horizontal_topic),
            "-p", "vertical_topic:={}".format(vertical_topic),
            "-p", "output_topic:={}".format(output_topic),
            "-p", "publish_tf:=false",
        ], env, args.output_dir / "composer.log", processes, logs)
        time.sleep(2)
        recorder = start([
            "ros2", "bag", "record", "--storage", "mcap", "-o", str(recorded_bag),
            output_topic, horizontal_topic, vertical_topic,
            "/vio/vertical_gate/diagnostics", "/vio/twist_gate/diagnostics",
        ], env, args.output_dir / "record.log", processes, logs)
        time.sleep(2)
        player = start([
            "ros2", "run", "pm_evaluation", "bag_clock_player", str(args.bag),
            "--rate", str(args.rate),
            *sum((["--topic", topic] for topic in DEFAULT_INPUT_TOPICS), []),
        ], env, args.output_dir / "play.log", processes, logs)
        if player.wait() != 0:
            raise RuntimeError("Clock player failed; see play.log")
        processes.remove(player)
        time.sleep(2)
        terminate(recorder)
        processes.remove(recorder)
    finally:
        for process in reversed(processes):
            terminate(process)
        for stream in logs:
            stream.close()

    composed = load_odometry(recorded_bag, output_topic)
    if not composed:
        raise RuntimeError("No composed output; inspect logs")
    horizontal = load_odometry(recorded_bag, horizontal_topic)
    vertical = load_odometry(recorded_bag, vertical_topic)
    vio = load_odometry(args.bag, "/vio/odometry")
    recorded = load_odometry(args.bag, "/odometry/local")
    write_csv(args.output_dir / "composed_local.csv", composed)
    write_csv(args.output_dir / "horizontal_ekf.csv", horizontal)
    write_csv(args.output_dir / "attitude_height_observer.csv", vertical)
    write_csv(args.output_dir / "vio_input.csv", vio)
    write_csv(args.output_dir / "recorded_local.csv", recorded)
    plot_results(args.output_dir / "separated_localization_comparison.png", composed, vio, recorded)
    plot_safe_output(args.output_dir / "separated_localization_safe_output.png",
                     composed, vertical)
    summary = {
        "input_bag": str(args.bag),
        "playback_rate": args.rate,
        "composed_local": safe_output_summary(composed),
        "horizontal_ekf": safe_output_summary(horizontal),
        "attitude_height_observer": series_summary(vertical),
        "vio_input": series_summary(vio),
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
