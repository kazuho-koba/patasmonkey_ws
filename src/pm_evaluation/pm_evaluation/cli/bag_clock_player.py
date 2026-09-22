#!/usr/bin/env python3
"""Replay selected MCAP topics while publishing the recorded ROS clock.

ROS 2 Foxy's ``ros2 bag play`` does not provide a ``--clock`` option.  This
small, intentionally typed player is used for deterministic EKF replays: the
clock follows each MCAP record timestamp and the original message is published
with its original header stamp.
"""

import argparse
import glob
import os
from pathlib import Path
import time
from typing import Dict, List, Tuple, Type

from mcap.reader import make_reader
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


MESSAGE_TYPES: Dict[str, Type] = {
    "/wheel/odometry": Odometry,
    "/vio/odometry": Odometry,
    "/wit/imu": Imu,
    "/tf_static": TFMessage,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", action="append", required=True)
    parser.add_argument("--rate", type=float, default=1.0)
    return parser.parse_args()


def read_records(bag: Path, topics: List[str]) -> List[Tuple[int, str, bytes]]:
    records: List[Tuple[int, str, bytes]] = []
    for mcap_path in sorted(glob.glob(str(bag / "*.mcap"))):
        with open(mcap_path, "rb") as stream:
            for _, channel, record in make_reader(stream).iter_messages():
                if channel.topic in topics:
                    records.append((record.log_time, channel.topic, record.data))
    return sorted(records, key=lambda item: item[0])


def main() -> None:
    args = parse_args()
    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")
    unknown = sorted(set(args.topic) - set(MESSAGE_TYPES))
    if unknown:
        raise SystemExit("Unsupported typed replay topic(s): {}".format(unknown))
    records = read_records(args.bag, args.topic)
    if not records:
        raise SystemExit("No selected messages found")

    rclpy.init()
    node = rclpy.create_node("bag_clock_player")
    regular_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
    static_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publishers = {
        topic: node.create_publisher(
            MESSAGE_TYPES[topic], topic, static_qos if topic == "/tf_static" else regular_qos
        )
        for topic in args.topic
    }
    clock_pub = node.create_publisher(Clock, "/clock", regular_qos)

    # Let subscriptions and transient-local /tf_static discovery settle.
    time.sleep(1.0)
    first_time = records[0][0]
    wall_start = time.monotonic()
    try:
        for log_time, topic, data in records:
            elapsed = (log_time - first_time) * 1e-9 / args.rate
            delay = wall_start + elapsed - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            sec, nanosec = divmod(log_time, 1_000_000_000)
            clock = Clock()
            clock.clock.sec = sec
            clock.clock.nanosec = nanosec
            clock_pub.publish(clock)
            publishers[topic].publish(deserialize_message(data, MESSAGE_TYPES[topic]))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
