#!/usr/bin/env python3
"""同一のbag・VIOゲートから分離方式と3D EKF２条件を同時に評価する。"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcap.reader import make_reader
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
import yaml
from ament_index_python.packages import get_package_share_directory

CONFIG = Path(get_package_share_directory('pm_config')) / 'config'
TOPICS = ['/wheel/odometry', '/wit/imu', '/vio/odometry', '/tf_static']
OUT = ['/comparison/separated', '/comparison/horizontal', '/comparison/vertical',
       '/comparison/ekf3d_no_accel', '/comparison/ekf3d_accel',
       '/comparison/accel_no_accel', '/comparison/accel_accel',
       '/vio/odometry/twist_gated', '/vio/vertical_gate/diagnostics',
       '/vio/twist_gate/diagnostics', '/vio/odometry/gated',
       '/comparison/projected', '/comparison/projected_horizontal',
       '/comparison/legacy',
       '/wheel/odometry/horizontal', '/vio/odometry/twist_horizontal',
       '/wit/imu/horizontal', '/localization/horizontal_projection/diagnostics']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--domain', default='153')
    parser.add_argument('--duration', type=float, help='先頭からの再生長さ [秒]')
    parser.add_argument('--heading-yaw', type=float, help='疑似初期化に使うENU yaw [rad]')
    parser.add_argument('--heading-stamp', type=float, help='初期化を実行するbag時間 [秒]')
    args = parser.parse_args()
    if (args.heading_yaw is None) != (args.heading_stamp is None):
        parser.error('--heading-yawと--heading-stampは同時に指定してください')
    args.bag = args.bag.resolve()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'replay_parameters.json').write_text(json.dumps({
        'bag':str(args.bag), 'rate':args.rate, 'duration':args.duration,
        'heading_yaw_rad':args.heading_yaw, 'heading_stamp':args.heading_stamp,
        'input_topics':TOPICS, 'heading_method':'set_poseによる疑似初期化' if args.heading_yaw is not None else '指定なし',
        'description':'記録済みVIOを再利用。local EKFのみ再計算。'}, ensure_ascii=False,indent=2)+'\n')
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = args.domain
    # 本番と同じ設定を保存する。比較のためTF配信だけを抑止する。
    config_paths = {}
    for name, filename, node in [('no_accel', 'ekf_local_3d_no_accel.yaml', 'ekf_local_3d_node'),
                                 ('accel', 'ekf_local_3d_accel.yaml', 'ekf_local_3d_node'),
                                 ('legacy', 'ekf_local_whl_imu_cam.yaml', 'ekf_local_node')]:
        config = yaml.safe_load((CONFIG / filename).read_text())
        config[node]['ros__parameters']['publish_tf'] = False
        config[node]['ros__parameters']['publish_acceleration'] = name != 'legacy'
        config_paths[name] = args.output / ('ekf_' + name + '.yaml')
        config_paths[name].write_text(yaml.safe_dump(config, sort_keys=False))
    processes = []
    files = []
    def start(cmd, name):
        f = (args.output / (name + '.log')).open('w')
        files.append(f)
        p = subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p
    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=10)
                raise RuntimeError('SIGINTで停止しません: {}'.format(p.args))
    try:
        for node in ['vio_vertical_gate_node', 'vio_twist_gate_node']:
            start(['ros2', 'run', 'pm_localization', node], node)
        config = CONFIG / 'ekf_local_horizontal_vio_twist.yaml'
        start(['ros2','run','robot_localization','ekf_node','--ros-args','--params-file',str(config),'-r','__node:=ekf_local_horizontal_node','-r','odometry/filtered:=/comparison/horizontal','-r','set_pose:=/comparison/set_pose/horizontal','-p','use_sim_time:=true'], 'horizontal')
        for name in ['no_accel', 'accel']:
            start(['ros2','run','robot_localization','ekf_node','--ros-args','--params-file',str(config_paths[name]),'-r','__node:=ekf_local_3d_node','-r','odometry/filtered:=/comparison/ekf3d_' + name,'-r','set_pose:=/comparison/set_pose/ekf3d_' + name,'-r','accel/filtered:=/comparison/accel_' + name,'-p','use_sim_time:=true'], name)
        start(['ros2','run','robot_localization','ekf_node','--ros-args','--params-file',str(config_paths['legacy']),'-r','__node:=ekf_local_node','-r','odometry/filtered:=/comparison/legacy','-r','set_pose:=/comparison/set_pose/legacy','-p','use_sim_time:=true'], 'legacy')
        start(['ros2','run','pm_localization','attitude_height_observer_node','--ros-args','-p','output_topic:=/comparison/vertical','-p','use_sim_time:=true'], 'vertical')
        start(['ros2','run','pm_localization','local_odometry_composer_node','--ros-args','-p','horizontal_topic:=/comparison/horizontal','-p','vertical_topic:=/comparison/vertical','-p','output_topic:=/comparison/separated','-p','publish_tf:=false'], 'composer')
        start(['ros2','run','pm_localization','horizontal_velocity_projection_node','--ros-args','-p','use_sim_time:=true'], 'projection')
        start(['ros2','run','robot_localization','ekf_node','--ros-args','--params-file',str(CONFIG/'ekf_local_horizontal_projected.yaml'),'-r','__node:=ekf_local_horizontal_node','-r','odometry/filtered:=/comparison/projected_horizontal','-r','set_pose:=/comparison/set_pose/projected_horizontal','-p','use_sim_time:=true'], 'projected_ekf')
        start(['ros2','run','pm_localization','local_odometry_composer_node','--ros-args','-r','__node:=projected_composer','-p','horizontal_topic:=/comparison/projected_horizontal','-p','vertical_topic:=/comparison/vertical','-p','output_topic:=/comparison/projected','-p','publish_tf:=false'], 'projected_composer')
        time.sleep(4)
        if any(p.poll() is not None for p in processes):
            raise RuntimeError('推定ノードの起動に失敗しました')
        recorder = start(['ros2','bag','record','--storage','mcap','-o',str(args.output/'replayed_bag'), *OUT], 'recorder')
        time.sleep(4)
        seed = None
        if args.heading_yaw is not None:
            seed = start(['ros2','run','pm_evaluation','seed_local_comparison','--yaw',str(args.heading_yaw),'--stamp',str(args.heading_stamp)], 'heading_seed')
        command = ['ros2','run','pm_evaluation','bag_clock_player',str(args.bag),'--rate',str(args.rate),'--drain-seconds','2', *sum((['--topic', t] for t in TOPICS), [])]
        if args.duration:
            command += ['--duration', str(args.duration)]
        player = start(command, 'player')
        while player.poll() is None:
            if seed and seed.poll() is not None:
                if seed.returncode:
                    raise RuntimeError('疑似方位初期化失敗。heading_seed.logを確認してください')
                processes.remove(seed)
                seed = None
            if any(p.poll() is not None for p in processes if p is not player and p is not seed):
                raise RuntimeError('再生中に推定ノードまたはrecorderが終了しました')
            time.sleep(.5)
        if player.returncode:
            raise RuntimeError('bag再生失敗')
        if seed is not None:
            raise RuntimeError('疑似方位初期化が完了しませんでした')
        time.sleep(2)
        stop(recorder)
        processes.remove(recorder)
    finally:
        for p in reversed(processes):
            stop(p)
        for f in files:
            f.close()
    subprocess.check_call(['ros2','bag','info',str(args.output/'replayed_bag')], env=env, stdout=subprocess.DEVNULL)
    # header stampを共通時間軸に用い、出力姿勢と共分散も保存する。
    series = {t: [] for t in OUT[:5] + ['/comparison/projected', '/comparison/legacy']}
    for path in sorted((args.output/'replayed_bag').glob('*.mcap')):
        with path.open('rb') as f:
            for _, channel, record in make_reader(f).iter_messages():
                if channel.topic not in series:
                    continue
                m = deserialize_message(record.data, Odometry)
                pos, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
                series[channel.topic].append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, m.pose.covariance[0], m.pose.covariance[7], m.pose.covariance[14], m.twist.covariance[14]])
    columns = ['stamp','x','y','z','qx','qy','qz','qw','vx','vy','vz','var_x','var_y','var_z','var_vz']
    for topic, rows in series.items():
        with (args.output/(topic.rsplit('/',1)[-1]+'.csv')).open('w') as f:
            w = csv.writer(f); w.writerow(columns); w.writerows(rows)
    selected = [('/comparison/legacy','legacy'),('/comparison/separated','separated'),
                ('/comparison/projected','projected'),('/comparison/ekf3d_no_accel','3D no accel'),
                ('/comparison/ekf3d_accel','3D accel')]
    if any(not series[t] for t, _ in selected):
        raise RuntimeError('比較出力が空です。ログとTFを確認してください')
    origin = min(series[t][0][0] for t,_ in selected)
    fig, ax = plt.subplots(2, 2, figsize=(13, 10))
    summary = {}
    for topic, label in selected:
        a = series[topic]
        ts = [r[0]-origin for r in a]
        ax[0,0].plot([r[1] for r in a], [r[2] for r in a], label=label)
        ax[0,1].plot(ts, [r[3] for r in a], label=label)
        ax[1,0].plot(ts, [r[10] for r in a], label=label)
        ax[1,1].plot(ts, [max(r[13],1e-12) for r in a], label=label)
        jumps = [abs(b[3]-a0[3]) for a0,b in zip(a,a[1:]) if b[0]>a0[0]]
        summary[label] = {'count':len(a), 'duration_s':a[-1][0]-a[0][0], 'final_xyz_m':a[-1][1:4], 'z_range_m':max(r[3] for r in a)-min(r[3] for r in a), 'max_z_step_m':max(jumps, default=0), 'final_z_variance':a[-1][13]}
    ax[0,0].set(xlabel='x [m]',ylabel='y [m]',title='Local trajectories'); ax[0,0].axis('equal')
    ax[0,1].set(xlabel='time [s]',ylabel='z [m]',title='Height')
    ax[1,0].set(xlabel='time [s]',ylabel='vz [m/s]',title='Vertical velocity')
    ax[1,1].set(xlabel='time [s]',ylabel='z variance [m²]',title='Reported z variance'); ax[1,1].set_yscale('log')
    for a in ax.flat: a.grid(); a.legend()
    fig.tight_layout(); fig.savefig(args.output/'comparison.png', dpi=160)
    (args.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))

if __name__ == '__main__':
    main()
