"""記録画像・IMUをOpenVINSへ再生し、bag時刻でraw出力を記録する。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from pm_evaluation.localization_batch_io import messages
from pm_evaluation.localization_vio_replay import BagWriter, schemas, SENSOR_TOPICS, validate_bag


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('job',type=Path)
    args=parser.parse_args();job=json.loads(args.job.read_text());folder=args.job.parent
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import Image,Imu
    from rosgraph_msgs.msg import Clock
    rclpy.init(args=[])
    node=rclpy.create_node('batch_vio_replay')
    writer=BagWriter(folder/'raw_bag',schemas(Path(job['bag'])))
    current={'ns':None};counts={};process=None
    def output(msg,topic,typename):
        if current['ns'] is not None:
            writer.write(current['ns'],topic,typename,serialize_message(msg))
            counts[topic]=counts.get(topic,0)+1
    qos=QoSProfile(depth=500,reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Odometry,'/ov_msckf/odomimu',
        lambda m:output(m,'/ov_msckf/odomimu','nav_msgs/msg/Odometry'),qos)
    node.create_subscription(DiagnosticArray,'/ov_msckf/diagnostics/health',
        lambda m:output(m,'/ov_msckf/diagnostics/health','diagnostic_msgs/msg/DiagnosticArray'),qos)
    clock=node.create_publisher(Clock,'/clock',qos)
    publishers={t:node.create_publisher(Imu if t.endswith('/data') else Image,t,qos) for t in SENSOR_TOPICS}
    stream=(folder/'openvins.log').open('w')
    try:
        process=subprocess.Popen(['ros2','run','ov_msckf','run_subscribe_msckf','--ros-args',
                '-r','__ns:=/ov_msckf','-p','use_sim_time:=true',
                '-p','config_path:='+job['config'],'-p','verbosity:=WARNING',
                '-p','save_total_state:=true','-p','filepath_est:='+str(folder/'state_estimate.txt'),
                '-p','filepath_std:='+str(folder/'state_std.txt')],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+30
        while not all(pub.get_subscription_count()>0 for pub in publishers.values()):
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()>deadline or process.poll() is not None:raise RuntimeError('OpenVINS画像/IMU購読の起動失敗')
        time.sleep(1)
        first=None;wall=None;sent={};lateness=0.
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
            sent[topic]=sent.get(topic,0)+1
            rclpy.spin_once(node,timeout_sec=0)
            if process.poll() is not None:raise RuntimeError('OpenVINSが再生中に終了')
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
        (folder/'delivery.json').write_text(json.dumps(dict(sent=sent,output=counts,maximum_lateness_sec=lateness,
             record_clock='出力callbackを受信した時点の最後の入力bag時刻。wall clockへ置換しない'),ensure_ascii=False,indent=2))
        if not counts.get('/ov_msckf/odomimu'):raise RuntimeError('raw VIO出力なし。bag内の初期化励起不足などをopenvins.logで確認')
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
                writer.close();stream.close();node.destroy_node();rclpy.shutdown()
                raise RuntimeError('OpenVINSのSIGINT終了失敗')
        writer.close();stream.close();node.destroy_node();rclpy.shutdown()
        if process is not None and process.returncode not in (0,130,-signal.SIGINT):
            raise RuntimeError('OpenVINS終了異常: '+str(process.returncode))
    validate_bag(folder/'raw_bag')


if __name__=='__main__':main()
