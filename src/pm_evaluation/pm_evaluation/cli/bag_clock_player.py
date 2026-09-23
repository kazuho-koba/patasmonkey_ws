#!/usr/bin/env python3
"""記録済みROS clockをpublishしつつ、選択したMCAP topicをreplayする。

ROS 2 Foxyの`ros2 bag play`には`--clock` optionがない。この小さく意図的に型を限定した
playerは決定的なEKF replayに用いる。clockは各MCAP record timestampに追従し、元messageは
元のheader stampのままpublishする。recordをmemoryへ蓄積せずMCAPからstreamするため、
JetsonでもRGB-D playbackを実行可能に保つ。
"""

import argparse
import glob
from pathlib import Path
import time
from typing import Dict, Iterable, List, Tuple, Type

from mcap.reader import make_reader
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_msgs.msg import TFMessage


MESSAGE_TYPES: Dict[str, Type] = {
    "/wheel/odometry": Odometry,
    "/vio/odometry": Odometry,
    "/wit/imu": Imu,
    "/tf_static": TFMessage,
    "/oak/depth/image_raw": Image,
    "/oak/depth/camera_info": CameraInfo,
    "/oak/color/image_raw": Image,
    "/oak/color/camera_info": CameraInfo,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", action="append", required=True)
    parser.add_argument("--rate", type=float, default=1.0)
    return parser.parse_args()


def iter_records(bag: Path, topics: List[str]) -> Iterable[Tuple[int, str, bytes]]:
    """画像dataを保持せず、MCAP file順に選択recordをyieldする。"""
    for mcap_path in sorted(glob.glob(str(bag / "*.mcap"))):
        with open(mcap_path, "rb") as stream:
            for _, channel, record in make_reader(stream).iter_messages():
                if channel.topic in topics:
                    yield record.log_time, channel.topic, record.data


def main() -> None:
    args = parse_args()
    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")
    # 明示的なwhitelistによりFoxyでのdeserializeを予測可能にし、replay commandがcontrol
    # topicを誤ってpublishすることを防ぐ。
    unknown = sorted(set(args.topic) - set(MESSAGE_TYPES))
    if unknown:
        raise SystemExit("Unsupported typed replay topic(s): {}".format(unknown))
    rclpy.init()
    node = rclpy.create_node("bag_clock_player")
    regular_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
    static_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    # `/tf_static`はreplay後に起動するsubscriberもcamera/base transformを受け取れるよう
    # transient-localにする。他のreplay dataはoffline evaluatorに合わせた通常のbounded
    # reliable trafficである。
    publishers = {
        topic: node.create_publisher(
            MESSAGE_TYPES[topic], topic, static_qos if topic == "/tf_static" else regular_qos
        )
        for topic in args.topic
    }
    clock_pub = node.create_publisher(Clock, "/clock", regular_qos)

    # 最初のsimulated timestampより前にsubscriptionとtransient-local `/tf_static` discoveryを
    # 落ち着かせる。元bagを変更せず、短いoffline runでの初期depth/TF raceを避ける。
    time.sleep(1.0)
    first_time = None
    wall_start = None
    try:
        for log_time, topic, data in iter_records(args.bag, args.topic):
            if first_time is None:
                first_time = log_time
                wall_start = time.monotonic()
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
        if first_time is None:
            raise SystemExit("No selected messages found")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
