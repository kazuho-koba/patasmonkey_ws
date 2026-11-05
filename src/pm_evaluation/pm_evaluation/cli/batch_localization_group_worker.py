"""１回の低帯域bag再生で独立した複数のlocal/global推定graphを評価する。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from pm_evaluation.localization_storage import recorder_environment

from pm_evaluation.localization_batch_io import messages
from pm_evaluation.localization_vio_replay import validate_bag


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('group',type=Path)
    args=parser.parse_args();job=json.loads(args.group.read_text());folder=args.group.parent
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from robot_localization.srv import SetPose
    rclpy.init(args=[]);node=rclpy.create_node('batch_shared_player')
    processes=[];streams=[];started=time.monotonic()

    def start(command,label):
        stream=(folder/(label+'.log')).open('w');streams.append(stream)
        p=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,
                           env=recorder_environment() if command[:3]==['ros2','bag','record'] else None,start_new_session=True)
        processes.append(p);return p

    def stop(p):
        if p.poll() is not None:return
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
            raise RuntimeError('SIGINT終了失敗: '+str(p.args))
        accepted=(0,2,130,-signal.SIGINT) if p.args[:3]==['ros2','bag','record'] else (0,130,-signal.SIGINT)
        if p.returncode not in accepted:raise RuntimeError('終了異常: '+str(p.args))

    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
    static=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    clock=node.create_publisher(Clock,'/clock',qos);publishers={}
    try:
        expected=set()
        for i,spec in enumerate(job['nodes']):
            expected.add((spec['name'],spec['namespace']))
            start(['ros2','run',spec['package'],spec['executable'],'--ros-args',
                   '--params-file',spec['parameters'],'-r','__node:='+spec['name'],
                   '-r','__ns:='+spec['namespace'],
                   *sum((['-r',source+':='+target] for source,target in spec['remaps']),[])],
                  str(i)+'_'+spec['name'])
        recorder=start(['ros2','bag','record','-s','mcap','-o',str(folder/'output_bag'),*job['outputs']],'recorder')
        clients=[node.create_client(SetPose,s) for s in job['services']]
        deadline=time.monotonic()+60
        # EKFサービスだけでなく全nodeのgraph登録も待つ。比較方式が多い場合の
        # Python import遅延を、初期入力の欠落へ変えない。
        while not (all(c.service_is_ready() for c in clients) and
                   expected.issubset(set(node.get_node_names_and_namespaces()))):
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()>deadline or any(p.poll() is not None for p in processes):
                raise RuntimeError('共有graphの起動待ち失敗')
        for c in clients:node.destroy_client(c)
        time.sleep(3)
        first=last=None;wall=None;counts={};lateness=0.;play_start=time.monotonic()
        for ns,topic,msg in messages(Path(job['bag']),job['input_topics']):
            if first is None:first=ns;wall=time.monotonic()
            if job['duration'] and (ns-first)*1e-9>job['duration']:break
            if topic not in publishers:
                publishers[topic]=node.create_publisher(type(msg),job['input_remaps'][topic],static if topic=='/tf_static' else qos)
                connected=time.monotonic();deadline=connected+30
                expected=job['expected_subscriptions'][topic]
                while publishers[topic].get_subscription_count()<expected:
                    rclpy.spin_once(node,timeout_sec=.02)
                    if time.monotonic()>deadline:
                        raise RuntimeError('入力subscriber接続待ち失敗: '+topic)
                # 接続待ちの壁時計時間を再生スケジュールへ織り込み、入力時刻は変えない。
                wall+=time.monotonic()-connected
            delay=wall+(ns-first)*1e-9/job['rate']-time.monotonic()
            if delay>0:time.sleep(delay)
            else:lateness=max(lateness,-delay)
            tick=Clock();tick.clock.sec,tick.clock.nanosec=divmod(ns,1000000000)
            clock.publish(tick);publishers[topic].publish(msg)
            counts[topic]=counts.get(topic,0)+1;last=ns
            rclpy.spin_once(node,timeout_sec=0)
            if any(p.poll() is not None for p in processes):raise RuntimeError('再生中に比較nodeが終了')
        if first is None:raise RuntimeError('共有入力が空')
        time.sleep(2);stop(recorder);processes.remove(recorder)
        (folder/'delivery.json').write_text(json.dumps(dict(branches=len(job['branches']),published=counts,
            first_record_ns=first,last_record_ns=last,maximum_lateness_sec=lateness,
            playback_wall_sec=time.monotonic()-play_start,
            wall_sec_including_startup=time.monotonic()-started),indent=2)+'\n')
    finally:
        errors=[]
        for p in reversed(processes):
            try:stop(p)
            except Exception as exc:errors.append(str(exc))
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown()
        if errors:raise RuntimeError('; '.join(errors))
    validate_bag(folder/'output_bag')


if __name__=='__main__':main()
