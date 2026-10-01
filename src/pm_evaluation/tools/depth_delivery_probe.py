"""mapperと同じQoSの軽量raw受信を、独立processで比較する診断専用probe。

TF・grid・hazardを処理せず、serialized ImageのHeader.stampだけCSVへ保存する。
これはmapperのDDS readerそのものではない。probeが10 Hzでもmapperのreader内で
何が起きたかは確定しないが、同じhostの別受信processで欠落が再現するかを調べられる。
"""
import argparse
import csv
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from pm_evaluation.cli.analyze_mapper_depth_delivery import image_header_stamp_ns


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--compare-reliable", action="store_true",
                        help="同じprocess/executorにRELIABLE履歴5のraw受信も追加する")
    args = parser.parse_args()
    rclpy.init()
    node = Node("pm_depth_delivery_probe", start_parameter_services=False,
                enable_rosout=False)
    # exclusive作成で過去結果を保護する。1秒batch flushで画像毎のdisk I/Oを避ける。
    stream = Path(args.output).open("x", newline="", buffering=262144)
    writer = csv.writer(stream)
    writer.writerow(["message_stamp_ns", "callback_monotonic_ns", "callback_wall_ns", "qos_label"])
    stream.flush()

    def callback(serialized, qos_label):
        writer.writerow([image_header_stamp_ns(serialized), time.monotonic_ns(), time.time_ns(), qos_label])

    # mapperと同じBEST_EFFORT / VOLATILE / KEEP_LAST 5。raw=Trueなので
    # Python Image objectや巨大なdata配列へのdeserializeは行わない。
    node.create_subscription(Image, "/oak/depth/image_raw", lambda data: callback(data, "best_effort"),
                             qos_profile_sensor_data, raw=True)
    if args.compare_reliable:
        # 同じ軽量executor・履歴5でreliabilityだけを変える。mapperのQoSは変更しない。
        node.create_subscription(Image, "/oak/depth/image_raw", lambda data: callback(data, "reliable"),
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE), raw=True)
    node.create_timer(1.0, stream.flush)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        stream.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
