"""共通入力を共有し、推定・TF・サービスだけを方式ごとに隔離する接続計画。"""
import json
from pathlib import Path
import signal
import subprocess
import os
import uuid
import time
from datetime import datetime

import yaml

from pm_evaluation.localization_vio_replay import validate_bag
from pm_evaluation.localization_batch_cache import result_ready
from pm_evaluation.localization_storage import split_and_verify,cleanup_shared

# 製品node内の絶対topic名も漏らさずremapする。frame名(odom/base_link等)は
# 物理的意味を維持し、TFの配送topicだけを各graphへ分離する。
INTERNAL = {'/tf','/diagnostics','/gps/filtered','/wheel/odometry/horizontal',
            '/vio/odometry/twist_horizontal','/wit/imu/horizontal',
            '/localization/horizontal_projection/diagnostics'}


def shared_job(jobs, group_folder):
    """同じ入力・再生区間の５方式までを１つの実行計画へ変換する。"""
    group_folder=Path(group_folder);group_folder.mkdir(parents=True,exist_ok=True)
    first=jobs[0][0]
    for job,_ in jobs:
        for key in ('bag','rate','duration','input_topics'):
            if job[key]!=first[key]:raise ValueError('共有できない再生条件: '+key)
    common={topic:'/evaluation/input'+topic for topic in first['input_topics']}
    # 静的TFは各方式共通。動的TFは絶対に他方式のlistenerへ渡さない。
    common['/tf_static']='/evaluation/input/tf_static'
    branches=[];specs=[];outputs=[];services=[]
    subscriptions={topic:0 for topic in first['input_topics']}
    for index,(job,folder) in enumerate(jobs):
        prefix='/evaluation/case_'+str(index)
        internal=INTERNAL|set(job['outputs'])|set(job['services'])
        internal|={target for spec in job['nodes'] for _,target in spec['remaps']}
        mapping={topic:prefix+topic for topic in internal if topic not in common}
        mapping.update(common)
        for spec in job['nodes']:
            name=spec['name'];namespace=prefix
            parameter_file=group_folder/(str(index)+'_'+name+'.yaml')
            original=yaml.safe_load((Path(folder)/spec['parameters']).read_text())
            values=original[name]['ros__parameters'];executable=spec['executable']
            # 初回publisher discoveryで１方式だけ最初の入力を失わないよう、
            # 接続計画から直接入力のsubscriber数を数える。
            used=[]
            if executable=='ekf_node':
                used=[v for k,v in values.items() if __import__('re').fullmatch(r'(odom|twist|imu)\d+',k)]
            elif executable=='vio_odom_adapter_node':used=[values.get('input_topic','/ov_msckf/odomimu')]
            elif executable=='attitude_height_observer_node':used=[values.get('imu_topic','/wit/imu')]
            elif executable=='horizontal_velocity_projection_node':used=['/wit/imu','/wheel/odometry']
            elif executable=='gnss_fix_gate_node':used=[values.get('input_fix_topic','/fix'),values.get('navpvt_topic','/navpvt')]
            if executable in ('ekf_node','navsat_transform_node','vio_odom_adapter_node'):used.append('/tf_static')
            for topic in used:
                if topic in subscriptions:subscriptions[topic]+=1
            parameter_file.write_text(yaml.safe_dump({namespace+'/'+name:original[name]},sort_keys=False))
            # 既存remapの右辺を先にスコープ化する。ROS remapの連鎖解決には依存しない。
            remaps=[(source,mapping.get(target,target)) for source,target in spec['remaps']]
            covered={source for source,_ in remaps}
            remaps += [(source,target) for source,target in sorted(mapping.items()) if source not in covered]
            # rclpy TransformListenerは相対名tf_staticを使う。絶対名だけの
            # remapではnamespace内の別topicになり、C++だけTFが届く状態になる。
            remaps += [('tf_static',common['/tf_static']),('tf',prefix+'/tf')]
            specs.append(dict(package=spec['package'],executable=spec['executable'],name=name,
                              namespace=namespace,parameters=str(parameter_file),remaps=remaps))
        selected={prefix+topic:topic for topic in job['outputs']}
        outputs.extend(selected)
        services += [mapping[s] for s in job['services']]
        branches.append(dict(folder=str(folder),prefix=prefix,outputs=selected))
    return dict(bag=first['bag'],duration=first['duration'],rate=first['rate'],
                input_topics=first['input_topics'],input_remaps=common,expected_subscriptions=subscriptions,nodes=specs,
                outputs=outputs,services=services,branches=branches)


def run_shared(jobs, root, args):
    """未完了の比較だけを共有再生。各方式のbagは元のtopic名へ戻して保存。"""
    status={};pending=[]
    for job,folder in jobs:
        marker=folder/'complete.json'
        if args.resume and result_ready(folder):
            status[str(folder)]='cached'
        else:pending.append((job,folder))
    if not pending:return status
    # 前回の異常終了記録は消さずに退避する。既存のmetadataへ上書きしない。
    for _,folder in pending:
        if (folder/'output_bag').exists():
            archive=folder/('incomplete_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
            archive.mkdir();__import__('shutil').move(str(folder/'output_bag'),str(archive/'output_bag'))
    group=Path(root)/('group_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    plan=shared_job(pending,group)
    (group/'group.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
    env=os.environ.copy();env['ROS_DOMAIN_ID']=str(args.domain);env['ROS_LOCALHOST_ONLY']='1'
    env['PM_EVALUATION_MCAP_COMPRESSION']=getattr(args,'mcap_compression','zstd')
    started=time.monotonic()
    with (group/'worker.log').open('w') as stream:
        process=subprocess.Popen(['ros2','run','pm_evaluation','batch_localization_group_worker',
                                  str(group/'group.json')],env=env,stdout=stream,stderr=subprocess.STDOUT)
        try:code=process.wait()
        except BaseException:
            process.send_signal(signal.SIGINT);process.wait(timeout=360);raise
    if code:raise RuntimeError('共有再生失敗: '+str(group/'worker.log'))
    worker_wall=time.monotonic()-started
    split_started=time.monotonic()
    group_bags=[group/'output_bag']
    verification=split_and_verify(group_bags,plan['branches'],group/'split_verification.json')
    for _,folder in pending:
        (folder/'execution.json').write_text(json.dumps(dict(mode='shared_replay',
            group=str(group/'group.json'),branches=len(pending),delivery=str(group/'delivery.json')),indent=2))
        status[str(folder)]='executed_shared'
    cleanup_shared(group,group_bags,verification,keep=getattr(args,'keep_shared_bags',False))
    (group/'performance.json').write_text(json.dumps(dict(worker_wall_sec=worker_wall,split_wall_sec=time.monotonic()-split_started,total_wall_sec=time.monotonic()-started),indent=2))
    return status
