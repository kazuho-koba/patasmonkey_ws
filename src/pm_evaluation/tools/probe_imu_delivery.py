#!/usr/bin/env python3
"""IMU publish後の配送とbag記録の欠測を切り分ける軽量ROS 2プローブ。

センサや車両を起動しない。既定でIMU二topicのheader/受信時刻だけをメモリに保持し、
SIGINT後にCSVへ保存する。画像を購読しないため画像転送帯域を増やさない。
追加購読の負荷はゼロではないので、A/Bとも同じ条件で使用する。
"""
import argparse
import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def probe_qos(depth, reliability):
    """取得済みheaderの比較用reader設定。未知値をBEへ黙って置換しない。"""
    if depth <= 0 or reliability not in ('best_effort', 'reliable'):
        raise ValueError('正のdepthと既知のreliabilityが必要です')
    return QoSProfile(depth=depth, reliability=(ReliabilityPolicy.RELIABLE
        if reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT))


class DeliveryProbe(Node):
    """typedの独立購読で、recorderと同時刻の到達状況を測る。既定BEを維持する。"""

    def __init__(self, depth, limit, odometry_topic='', reliability='best_effort', imu_topics=None):
        super().__init__('imu_delivery_probe'+('_reliable' if reliability == 'reliable' else ''))
        self.rows = []
        self.limit = limit
        self.evicted = 0
        self.subscriptions_owned = []
        # 200件は125 Hzで約1.6秒の局所的な配送遅延を収容する比較用値。
        # キューを満たすために送信側を待たせないBEST_EFFORTとし、制御には使わない。
        # 比較用RELIABLEも選択できる。同じtyped callbackでQoSだけを変え、
        # generic recorderとの差とmiddleware配送の差を区別する。既定BEは従来通り。
        qos = probe_qos(depth, reliability)
        for topic in (imu_topics or ('/oak/imu/data', '/wit/imu')):
            self.subscriptions_owned.append(self.create_subscription(
                Imu, topic, lambda msg, name=topic: self.receive(name, msg), qos))
        if odometry_topic:
            # 再生試験ではVIO出力も同じ形式で残し、保護発動後の出力停止を確認する。
            self.subscriptions_owned.append(self.create_subscription(
                Odometry, odometry_topic,
                lambda msg: self.receive(odometry_topic, msg), qos))

    def receive(self, topic, msg):
        """デシリアライズ済みheaderと受信時計を保存し、callback内ではI/Oしない。"""
        if len(self.rows) >= self.limit:
            self.evicted += 1
            return
        self.rows.append((topic, time.time(), time.monotonic(),
                          msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--depth', type=int, default=200)
    parser.add_argument('--max-samples', type=int, default=60000)
    parser.add_argument('--odometry-topic', default='', help='出力停止検証用の追加Odometry topic')
    parser.add_argument('--reliability', choices=['best_effort', 'reliable'], default='best_effort',
                        help='typed readerの比較用QoS。publisherやrecorderは変更しない')
    parser.add_argument('--imu-topic', action='append', choices=['/oak/imu/data', '/wit/imu'],
                        help='比較対象。省略時は従来通りIMU二topic')
    args = parser.parse_args()
    if args.depth <= 0 or args.max_samples <= 0:
        parser.error('depthとmax-samplesは正数で指定してください')
    if args.imu_topic and len(set(args.imu_topic)) != len(args.imu_topic):
        parser.error('同じIMU topicを重複指定できません')
    path = Path(args.output).expanduser()
    if path.exists():
        parser.error('既存の診断結果は上書きしません')
    rclpy.init(args=[])
    node = DeliveryProbe(args.depth, args.max_samples, args.odometry_topic,
                         args.reliability, args.imu_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('topic', 'receipt_wall_s', 'receipt_monotonic_s', 'header_s'))
            writer.writerows(node.rows)
        print('DELIVERY_SAVED', path, 'count=', len(node.rows),
              'discarded=', node.evicted, flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
