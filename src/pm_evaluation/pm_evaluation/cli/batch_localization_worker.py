"""隔離ROS domainで１方式を再計算する。実機・操作・制御ノードは起動しない。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml
from pm_evaluation.localization_storage import recorder_environment

from pm_evaluation.localization_batch_io import messages, header_sec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('job', type=Path)
    args = parser.parse_args()
    job = json.loads(args.job.read_text())
    folder = args.job.parent
    # 環境のdomain指定は親が設定する。既存実機graphのdomainとは共有しない。
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from rosgraph_msgs.msg import Clock
    from rosidl_runtime_py.utilities import get_message
    rclpy.init(args=[])
    node = rclpy.create_node('batch_localization_player')
    processes, streams = [], []

    def start(command, name):
        stream = (folder/(name+'.log')).open('w'); streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   env=recorder_environment() if command[:3]==['ros2','bag','record'] else None,
                                   start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=20)
            if process.returncode not in ((0,2,130,-signal.SIGINT) if process.args[:3]==['ros2','bag','record'] else (0,130,-signal.SIGINT)):
                raise RuntimeError('子プロセス終了異常: '+str(process.returncode)+' '+str(process.args))
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)
            raise RuntimeError('SIGINT終了失敗: '+str(process.args))

    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    clock = node.create_publisher(Clock, '/clock', qos)
    publishers = {}
    try:
        for spec in job['nodes']:
            start(['ros2', 'run', spec['package'], spec['executable'], '--ros-args',
                   '--params-file', str(folder/spec['parameters']), '-r', '__node:='+spec['name'],
                   *sum((['-r', a+':='+b] for a,b in spec['remaps']), [])], spec['name'])
        recorder = start(['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(folder/'output_bag'),
                          *job['outputs']], 'recorder')
        # discovery完了をサービスで確認する。単なる固定sleepだけに依存しない。
        from robot_localization.srv import SetPose
        clients = [node.create_client(SetPose, name) for name in job['services']]
        deadline = time.monotonic()+30
        while not all(c.service_is_ready() for c in clients):
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic()>deadline or any(p.poll() is not None for p in processes):
                raise RuntimeError('EKF起動待ち失敗')
        for c in clients:
            node.destroy_client(c)
        time.sleep(3)
        first = None; wall = None; count = {}; lateness = 0.; last = None
        for ns, topic, msg in messages(Path(job['bag']), job['input_topics']):
            if first is None:
                first = ns; wall = time.monotonic()
            if job['duration'] and (ns-first)*1e-9 > job['duration']:
                break
            if topic not in publishers:
                publishers[topic] = node.create_publisher(type(msg), topic,
                                                        static_qos if topic=='/tf_static' else qos)
                # 新しいpublisherの初回から落とさないためdiscovery時間を与える。
                time.sleep(.3)
                wall += .3
            target = wall+(ns-first)*1e-9/job['rate']
            delay = target-time.monotonic()
            if delay>0:
                time.sleep(delay)
            else:
                lateness=max(lateness,-delay)
            tick=Clock(); tick.clock.sec, tick.clock.nanosec=divmod(ns,1000000000)
            clock.publish(tick); publishers[topic].publish(msg)
            count[topic]=count.get(topic,0)+1; last=ns
            rclpy.spin_once(node, timeout_sec=0)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('再計算中に子プロセスが終了')
        if first is None:
            raise RuntimeError('再生対象が空')
        # 最終入力を処理し終える時間を確保。clockは進めず外挿を余分に追加しない。
        time.sleep(2)
        stop(recorder); processes.remove(recorder)
        (folder/'delivery.json').write_text(json.dumps({'published': count,
             'first_record_ns': first, 'last_record_ns': last,
             'maximum_lateness_sec': lateness}, indent=2)+'\n')
    finally:
        errors=[]
        for process in reversed(processes):
            try: stop(process)
            except Exception as exc: errors.append(str(exc))
        for stream in streams: stream.close()
        node.destroy_node(); rclpy.shutdown()
        if errors: raise RuntimeError('; '.join(errors))
    if not (folder/'output_bag/metadata.yaml').is_file():
        raise RuntimeError('記録metadataが生成されない')
    from pm_evaluation.localization_vio_replay import validate_bag
    validate_bag(folder/'output_bag')


if __name__ == '__main__':
    main()
