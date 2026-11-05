"""旧batch_vio_workerを基に、１つの画像/IMU再生を複数の独立OpenVINSへ配る。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from pm_evaluation.localization_batch_io import messages
from pm_evaluation.localization_vio_replay import BagWriter,schemas,SENSOR_TOPICS,validate_bag
from pm_evaluation.localization_batch_twopass import ResourceMonitor


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('job',type=Path)
    args=parser.parse_args();job=json.loads(args.job.read_text());folder=args.job.parent
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import Image,Imu
    from rosgraph_msgs.msg import Clock
    rclpy.init(args=[]);node=rclpy.create_node('batch_vio_twopass_replay')
    current={'ns':None};branches=[];streams=[];publishers={}
    qos=QoSProfile(depth=500,reliability=ReliabilityPolicy.RELIABLE)
    clock=node.create_publisher(Clock,'/clock',qos)
    monitor=ResourceMonitor();sent={};lateness=0.;started=time.monotonic();failure=None
    definitions=schemas(Path(job['bag']))
    input_names={t:'/evaluation/vo_input'+t for t in SENSOR_TOPICS}
    try:
        for index,spec in enumerate(job['variants']):
            target=Path(spec['folder']);target.mkdir(parents=True,exist_ok=True)
            state=dict(spec,writer=BagWriter(target/'raw_bag',definitions),counts={},process=None,error=None)
            branches.append(state)
            namespace='/evaluation/openvins_'+str(index)
            def receive(msg,topic,typename,state=state):
                if current['ns'] is not None:
                    state['writer'].write(current['ns'],topic,typename,serialize_message(msg))
                    state['counts'][topic]=state['counts'].get(topic,0)+1
            node.create_subscription(Odometry,namespace+'/odomimu',
                lambda m,cb=receive:cb(m,'/ov_msckf/odomimu','nav_msgs/msg/Odometry'),qos)
            node.create_subscription(DiagnosticArray,namespace+'/diagnostics/health',
                lambda m,cb=receive:cb(m,'/ov_msckf/diagnostics/health','diagnostic_msgs/msg/DiagnosticArray'),qos)
            stream=(target/'openvins.log').open('w');streams.append(stream)
            # 推定parameterは旧版と同じ。異なるのはnamespaceと入力配送先だけ。
            remaps=sum((['-r',source+':='+destination] for source,destination in input_names.items()),[])
            command=['ros2','run','ov_msckf','run_subscribe_msckf','--ros-args',
                '-r','__ns:='+namespace,'-p','use_sim_time:=true','-p','config_path:='+spec['config'],
                '-p','verbosity:=WARNING','-p','save_total_state:=true',
                '-p','filepath_est:='+str(target/'state_estimate.txt'),
                '-p','filepath_std:='+str(target/'state_std.txt'),*remaps]
            state['process']=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        publishers={t:node.create_publisher(Imu if t.endswith('/data') else Image,input_names[t],qos) for t in SENSOR_TOPICS}
        deadline=time.monotonic()+60
        # 両方のOpenVINSの購読成立を待ち、片側だけ初期画像が欠けることを防ぐ。
        while not all(p.get_subscription_count()>=len(branches) for p in publishers.values()):
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()>deadline or any(s['process'].poll() is not None for s in branches):
                raise RuntimeError('並列OpenVINSの全画像/IMU購読が成立しない')
        time.sleep(1)
        first=None;wall=None
        for ns,topic,msg in messages(Path(job['bag']),SENSOR_TOPICS):
            if first is None:first=ns;wall=time.monotonic()
            if job['duration'] and (ns-first)*1e-9>job['duration']:break
            target=wall+(ns-first)*1e-9/job['rate']
            while time.monotonic()<target:
                rclpy.spin_once(node,timeout_sec=min(.005,max(0.,target-time.monotonic())))
            lateness=max(lateness,time.monotonic()-target)
            current['ns']=ns
            tick=Clock();tick.clock.sec,tick.clock.nanosec=divmod(ns,1000000000)
            clock.publish(tick);publishers[topic].publish(msg)
            sent[topic]=sent.get(topic,0)+1;rclpy.spin_once(node,timeout_sec=0)
            monitor.sample([os.getpid()])
            if any(s['process'].poll() is not None for s in branches):
                raise RuntimeError('並列OpenVINSが再生中に終了')
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
    except Exception as exc:
        failure=str(exc)
    finally:
        # 一方の終了異常があっても、もう一方とwriterを必ず閉じる。
        for state in branches:
            process=state['process']
            if process is not None and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
                    state['error']='SIGINT終了失敗'
            if process is not None and process.returncode not in (0,130,-signal.SIGINT):
                state['error']='OpenVINS終了異常: '+str(process.returncode)
            if failure:state['error']=failure
            if not state['counts'].get('/ov_msckf/odomimu'):state['error']=state['error'] or 'raw VIO出力なし（初期化/診断を確認）'
            state['writer'].close()
            validate_bag(Path(state['folder'])/'raw_bag')
            (Path(state['folder'])/'delivery.json').write_text(json.dumps(dict(sent=sent,output=state['counts'],
                maximum_lateness_sec=lateness,shared_variants=len(branches),error=state['error'],
                record_clock='出力受信時点の最後の入力bag時刻。旧版と同じ定義'),ensure_ascii=False,indent=2))
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown()
        monitor.save(folder/'resources.json')
        (folder/'delivery.json').write_text(json.dumps(dict(replays=1,sent=sent,maximum_lateness_sec=lateness,
            wall_sec=time.monotonic()-started,variants=[dict(variant=s['variant'],folder=s['folder'],
            error=s['error'],output=s['counts']) for s in branches]),ensure_ascii=False,indent=2))
    # 条件別失敗はdeliveryに残す。親は成功条件だけで後段を構成しwheelは継続する。
    if failure:raise RuntimeError(failure)


if __name__=='__main__':main()
