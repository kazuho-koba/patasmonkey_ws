#!/usr/bin/env python3
"""bag再生中の最新/odometry/localを撮像時刻補間用CSVに保存する。

記録済みbagのodomは読まない。最新localizerが再計算した値を保存するだけで、
TFやセンサ・速度指令を発行しない。SIGINT時にファイルを閉じる。
"""
import argparse
import csv
import signal
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    # Foxyのrclpy signal handlerではspinが終了しない環境があるため、
    # このoffline collectorはPythonのKeyboardInterruptで確実にcloseへ進む。
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = rclpy.create_node("offline_recomputed_pose_recorder")
    try:
        with open(args.output, "w", newline="", buffering=1) as stream:
            writer = csv.writer(stream)
            writer.writerow(["stamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw"])

            def receive(message):
                p, q = message.pose.pose.position, message.pose.pose.orientation
                writer.writerow([message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                                 p.x, p.y, p.z, q.x, q.y, q.z, q.w])

            subscription = node.create_subscription(Odometry, "/odometry/local", receive, 100)
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
