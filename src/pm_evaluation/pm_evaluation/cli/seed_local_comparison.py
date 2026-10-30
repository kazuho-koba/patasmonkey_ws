"""過去試行の磁気初期化結果を複数EKFへ一度だけ再適用する。"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from robot_localization.srv import SetPose

from pm_localization.heading_initializer_node import rpy_from_quaternion, quaternion_from_rpy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--yaw', type=float, required=True)
    parser.add_argument('--stamp', type=float, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('comparison_heading_seed')
    modes = ['legacy', 'horizontal', 'projected_horizontal', 'ekf3d_no_accel', 'ekf3d_accel']
    topics = {name: '/comparison/' + name for name in modes}
    latest, futures = {}, {}
    clients = {name: node.create_client(SetPose, '/comparison/set_pose/' + name) for name in modes}
    subscriptions = []
    for name in modes:
        subscriptions.append(node.create_subscription(Odometry, topics[name],
            lambda msg, key=name: latest.__setitem__(key, msg), 20))
    state = {'time': None}
    def clock(msg):
        state['time'] = msg.clock.sec + msg.clock.nanosec * 1e-9
    subscriptions.append(node.create_subscription(Clock, '/clock', clock, 50))
    wall_deadline = time.monotonic() + 90.
    try:
        while rclpy.ok() and time.monotonic() < wall_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if state['time'] is None or state['time'] < args.stamp:
                continue
            for name in modes:
                if name in futures or name not in latest or not clients[name].service_is_ready():
                    continue
                odom = latest[name]
                # 本番と同じset_poseを使い、現時刻の位置・roll/pitchを維持しyawのみ置換する。
                roll, pitch, _ = rpy_from_quaternion(odom.pose.pose.orientation)
                x, y, z, w = quaternion_from_rpy(roll, pitch, args.yaw)
                request = SetPose.Request()
                request.pose.header = odom.header
                request.pose.pose = odom.pose
                request.pose.pose.pose.orientation.x = x
                request.pose.pose.pose.orientation.y = y
                request.pose.pose.pose.orientation.z = z
                request.pose.pose.pose.orientation.w = w
                request.pose.pose.covariance[35] = .122
                futures[name] = clients[name].call_async(request)
                print('seed_request {} {:.6f}'.format(name, state['time']), flush=True)
            if len(futures) == len(modes) and all(f.done() for f in futures.values()):
                for name, future in futures.items():
                    future.result()
                    print('seed_ok {}'.format(name), flush=True)
                return
            if state['time'] > args.stamp + 2.:
                raise RuntimeError('初期化期限超過: {}'.format([k for k in modes if k not in futures or not futures[k].done()]))
        raise RuntimeError('初期化clock待ちtimeout')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
