"""旧共有workerを基に、１つのbag走査を複数domainの全EKF条件へ配信する。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from pm_evaluation.localization_batch_io import messages
from pm_evaluation.localization_vio_replay import validate_bag
from pm_evaluation.localization_batch_twopass import ResourceMonitor
from pm_evaluation.localization_storage import recorder_environment


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('group',type=Path)
    args=parser.parse_args();job=json.loads(args.group.read_text());folder=args.group.parent
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from robot_localization.srv import SetPose
    processes=[];streams=[];parts=[];started=time.monotonic();monitor=ResourceMonitor()
    base_domain=int(os.environ.get('ROS_DOMAIN_ID','192'))

    def start(command,label,offset):
        stream=(folder/(label+'.log')).open('w');streams.append(stream)
        env=os.environ.copy();env['ROS_DOMAIN_ID']=str(base_domain+offset)
        # 3x3/6x6行列を扱うPython nodeが各々BLAS threadを大量生成すると、
        # node起動だけで数千threadになる。NumPy内部だけを1 threadに固定する。
        # EKFモデル・OpenVINSのthread設定・数式は変更しない。
        env['OPENBLAS_NUM_THREADS']='1'
        if command[:3]==['ros2','bag','record']:env=recorder_environment(env)
        p=subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p

    def stop_all():
        # 全processへ先にSIGINTを送り、停止待ちをnode数倍にしない。
        for p in processes:
            if p.poll() is None:
                try:os.killpg(p.pid,signal.SIGINT)
                except ProcessLookupError:pass
        deadline=time.monotonic()+25;errors=[]
        for p in reversed(processes):
            try:p.wait(timeout=max(.1,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10);errors.append('SIGINT終了失敗: '+str(p.pid))
            accepted=(0,2,130,-signal.SIGINT) if p.args[:3]==['ros2','bag','record'] else (0,130,-signal.SIGINT)
            if p.returncode not in accepted:errors.append('終了異常: '+str(p.returncode)+' '+str(p.pid))
        return errors

    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
    static=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    try:
        for spec in job['partitions']:
            offset=spec['offset'];context=Context()
            # Foxyはinit(domain_id=...)を持たない。Context初期化時だけ環境を選ぶ。
            os.environ['ROS_DOMAIN_ID']=str(base_domain+offset)
            rclpy.init(args=[],context=context)
            node=rclpy.create_node('batch_twopass_player_'+str(offset),context=context)
            executor=SingleThreadedExecutor(context=context);executor.add_node(node)
            part=dict(spec,context=context,node=node,executor=executor,
                      clock=node.create_publisher(Clock,'/clock',qos),publishers={},counts={})
            parts.append(part)
        os.environ['ROS_DOMAIN_ID']=str(base_domain)
        for i,spec in enumerate(job['nodes']):
            start(['ros2','run',spec['package'],spec['executable'],'--ros-args',
                  '--params-file',spec['parameters'],'-r','__node:='+spec['name'],'-r','__ns:='+spec['namespace'],
                  *sum((['-r',source+':='+target] for source,target in spec['remaps']),[])],
                  str(i)+'_'+spec['name'],spec['domain_offset'])
        for part in parts:
            offset=part['offset'];part['recorder']=start(['ros2','bag','record','-s','mcap','-o',str(folder/('output_bag_'+str(offset))),*part['outputs']],
                                                       'recorder_'+str(offset),offset)
            part['clients']=[part['node'].create_client(SetPose,s) for s in part['services']]
            part['expected_nodes']={(s['name'],s['namespace']) for s in job['nodes'] if s['domain_offset']==offset}
        deadline=time.monotonic()+180
        while True:
            ready=True;missing={}
            for part in parts:
                part['executor'].spin_once(timeout_sec=0)
                nodes=part['expected_nodes']-set(part['node'].get_node_names_and_namespaces())
                services=sum(not c.service_is_ready() for c in part['clients'])
                if nodes or services:ready=False;missing[part['offset']]=dict(nodes=sorted(nodes),services=services)
            if ready:break
            monitor.sample([os.getpid()])
            if time.monotonic()>deadline or any(p.poll() is not None for p in processes):raise RuntimeError('domain別起動待ち失敗: '+str(missing))
            time.sleep(.05)
        for part in parts:
            for client in part['clients']:part['node'].destroy_client(client)
        time.sleep(3)
        first=last=None;wall=None;counts={};lateness=0.;wall_shift=0.;play_start=time.monotonic()
        # このiteratorは１つだけ。domainを分けても再生回数は増やさない。
        for ns,topic,msg in messages(Path(job['bag']),job['input_topics']):
            if first is None:first=ns;wall=time.monotonic()
            if job['duration'] and (ns-first)*1e-9>job['duration']:break
            active=[p for p in parts if p['expected_subscriptions'].get(topic,0)>0]
            connected=time.monotonic()
            for part in active:
                if topic not in part['publishers']:
                    part['publishers'][topic]=part['node'].create_publisher(type(msg),job['input_remaps'][topic],static if topic=='/tf_static' else qos)
                    deadline=time.monotonic()+30
                    while part['publishers'][topic].get_subscription_count()<part['expected_subscriptions'][topic]:
                        part['executor'].spin_once(timeout_sec=.01)
                        if time.monotonic()>deadline:raise RuntimeError('domain %dの入力接続待ち失敗: %s'%(part['offset'],topic))
            # 初回の接続待ちだけを壁時計scheduleへ足す。入力時刻は変更しない。
            if topic not in counts:wall+=time.monotonic()-connected
            delay=wall+(ns-first)*1e-9/job['rate']-time.monotonic()
            if delay>0:time.sleep(delay)
            else:
                lateness=max(lateness,-delay)
                # 処理停滞後に追い付きburstを作らない。bag/センサ時刻はそのまま、
                # 実時間scheduleだけを延ばす。延長量を性能記録へ明示する。
                if -delay>.005:
                    wall-=delay;wall_shift-=delay
            tick=Clock();tick.clock.sec,tick.clock.nanosec=divmod(ns,1000000000)
            for part in parts:part['clock'].publish(tick)
            for part in active:
                part['publishers'][topic].publish(msg);part['counts'][topic]=part['counts'].get(topic,0)+1
            counts[topic]=counts.get(topic,0)+1;last=ns
            # playerはpublish専用。不要なdomainごとのwait-set構築を毎message行わない。
            monitor.sample([os.getpid()])
            if any(p.poll() is not None for p in processes):raise RuntimeError('共有再生中にnodeが終了')
        if first is None:raise RuntimeError('共有入力が空')
        time.sleep(2)
        (folder/'delivery.json').write_text(json.dumps(dict(replays=1,branches=len(job['branches']),published=counts,
            domains=[dict(offset=p['offset'],domain=base_domain+p['offset'],published=p['counts']) for p in parts],
            first_record_ns=first,last_record_ns=last,maximum_lateness_sec=lateness,
            schedule_extension_sec=wall_shift,playback_wall_sec=time.monotonic()-play_start,wall_sec_including_startup=time.monotonic()-started),indent=2))
    finally:
        errors=stop_all()
        for stream in streams:stream.close()
        for part in parts:
            part['executor'].shutdown();part['node'].destroy_node();part['context'].shutdown()
        monitor.save(folder/'resources.json')
        if errors:raise RuntimeError('; '.join(errors))
    for part in parts:validate_bag(folder/('output_bag_'+str(part['offset'])))


if __name__=='__main__':main()
