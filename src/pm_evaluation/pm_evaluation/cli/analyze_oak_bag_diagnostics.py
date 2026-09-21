#!/usr/bin/env python3
"""Summarize OAK device sequence continuity recorded in an MCAP rosbag."""

import argparse
import glob
from collections import defaultdict
from pathlib import Path
from typing import DefaultDict, Dict, Iterable, List, Tuple

from diagnostic_msgs.msg import DiagnosticArray
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message


TOPICS = {
    "/oak/diagnostics/left_frame": "left",
    "/oak/diagnostics/right_frame": "right",
    "/oak/diagnostics/color_frame": "color",
    "/oak/diagnostics/depth_frame": "depth",
    "/oak/diagnostics/imu_packet": "imu",
}


def read_diagnostics(bag_dir: Path) -> DefaultDict[str, List[Dict[str, str]]]:
    """Read OAK DiagnosticArray values from every MCAP segment in *bag_dir*."""
    diagnostics: DefaultDict[str, List[Dict[str, str]]] = defaultdict(list)
    pattern = str(bag_dir / "*.mcap")
    for mcap_path in sorted(glob.glob(pattern)):
        with open(mcap_path, "rb") as stream:
            for _, channel, record in make_reader(stream).iter_messages():
                stream_name = TOPICS.get(channel.topic)
                if stream_name is None:
                    continue
                message = deserialize_message(record.data, DiagnosticArray)
                if message.status:
                    diagnostics[stream_name].append({
                        value.key: value.value
                        for value in message.status[0].values
                    })
    return diagnostics


def sequence_steps(sequences: Iterable[int]) -> List[int]:
    """Return unsigned 32-bit increments, including correct wrap-around."""
    sequence_list = list(sequences)
    return [
        (current - previous) & 0xffffffff
        for previous, current in zip(sequence_list, sequence_list[1:])
    ]


def print_frame_report(name: str, rows: List[Dict[str, str]]) -> None:
    """Print sequence and device-clock continuity for one image stream."""
    sequences = [int(row["sequence_num"]) for row in rows]
    device_times = [int(row["device_timestamp_ns"]) for row in rows]
    steps = sequence_steps(sequences)
    time_steps = [
        current - previous
        for previous, current in zip(device_times, device_times[1:])
    ]
    gaps: List[Tuple[int, int, int]] = [
        (index, step, time_steps[index])
        for index, step in enumerate(steps)
        if step != 1
    ]
    print("[{}] messages={}".format(name, len(rows)))
    print(
        "  sequence: first={}, last={}, missing={}, nonunit_steps={}, "
        "max_step={}".format(
            sequences[0], sequences[-1],
            sum(step - 1 for step in steps if step > 1),
            len(gaps), max(steps, default=0),
        )
    )
    print(
        "  device interval: median={} ns, max={} ns, over_75ms={}".format(
            sorted(time_steps)[len(time_steps) // 2],
            max(time_steps, default=0),
            sum(step > 75_000_000 for step in time_steps),
        )
    )
    if gaps:
        print("  largest sequence gaps: {}".format(
            sorted(gaps, key=lambda item: item[1], reverse=True)[:8]
        ))


def print_imu_report(rows: List[Dict[str, str]]) -> None:
    """Print separate accelerometer and gyroscope sequence continuity."""
    print("[imu] messages={}".format(len(rows)))
    for key in ("accelerometer_sequence_num", "gyroscope_sequence_num"):
        sequences = [int(row[key]) for row in rows if key in row]
        if not sequences:
            print("  {}: unavailable".format(key))
            continue
        steps = sequence_steps(sequences)
        print(
            "  {}: first={}, last={}, missing={}, nonunit_steps={}, "
            "max_step={}".format(
                key, sequences[0], sequences[-1],
                sum(step - 1 for step in steps if step > 1),
                sum(step != 1 for step in steps), max(steps, default=0),
            )
        )


def main() -> None:
    """Run the command-line diagnostic report."""
    parser = argparse.ArgumentParser(
        description=(
            "Read /oak/diagnostics/* topics in an MCAP rosbag and report "
            "device-side image/IMU sequence continuity."
        )
    )
    parser.add_argument("bag_dir", type=Path, help="rosbag2 directory")
    args = parser.parse_args()
    if not args.bag_dir.is_dir():
        parser.error("not a directory: {}".format(args.bag_dir))

    diagnostics = read_diagnostics(args.bag_dir)
    if not diagnostics:
        parser.error("no /oak/diagnostics/* messages found")
    for name in ("left", "right", "color", "depth"):
        rows = diagnostics.get(name, [])
        if rows:
            print_frame_report(name, rows)
    if diagnostics.get("imu"):
        print_imu_report(diagnostics["imu"])


if __name__ == "__main__":
    main()
