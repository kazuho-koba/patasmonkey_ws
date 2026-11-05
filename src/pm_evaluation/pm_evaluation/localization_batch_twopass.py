"""２回再生版の入力hub、条件隔離、低負荷のprocess資源記録。"""
import heapq
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from pm_evaluation.localization_batch_io import records
from pm_evaluation.localization_vio_replay import BagWriter,schemas,validate_bag

COMMON_TOPICS=['/wheel/odometry','/wit/imu','/fix','/navpvt','/tf_static']
RAW_TOPIC='/ov_msckf/odomimu'


def variant_topic(variant):
    """ROS topic名ではhyphenを使えないため、保存directory名と配送名を分ける。"""
    return '/evaluation/vo_records/'+variant.replace('-','_')+'/odomimu'


def attach_hub(job,hub,variant):
    """共通入力は共有し、VO入力だけを条件ごとのhub channelへ切り替える。"""
    job=dict(job);job['bag']=str(hub)
    job['input_aliases']={} if job['wheel_only'] else {RAW_TOPIC:variant_topic(variant)}
    job['input_topics']=[job['input_aliases'].get(t,t) for t in job['input_topics']]
    return job


def build_hub(original,lowband,variants,output):
    """共通入力を１回と、各条件のraw VOをCDRのまま時刻順に合成する。

    再生待ちのないファイル処理。入力timestamp/header/内容を変更しない。
    variantsの値はraw VOを含むbagで、記録条件と再計算条件を混ぜない。
    """
    definitions=schemas(original)
    for bag in variants.values():definitions.update(schemas(bag))
    writer=BagWriter(output,definitions)
    def renamed(bag,variant):
        for ns,topic,name,raw in records(bag,[RAW_TOPIC]):yield ns,variant_topic(variant),name,raw
    streams=[records(lowband,COMMON_TOPICS)]
    streams += [renamed(bag,variant) for variant,bag in sorted(variants.items())]
    try:
        for ns,topic,name,raw in heapq.merge(*streams,key=lambda row:row[0]):writer.write(ns,topic,name,raw)
    finally:writer.close()
    validate_bag(output)


def run_worker(command,folder,domain):
    """親が制御sessionを保ち、異常時もworkerの子終了処理を待つ。"""
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1')
    with (Path(folder)/'worker.log').open('w') as stream:
        process=subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT)
        try:code=process.wait()
        except BaseException:
            process.send_signal(signal.SIGINT);process.wait(timeout=360);raise
    if code:raise RuntimeError('worker失敗: '+str(Path(folder)/'worker.log'))


class ResourceMonitor:
    """追加ROS購読を作らず、5秒に一度/procのメモリ・CPU tickを記録する。"""
    def __init__(self):self.samples=[];self.previous=0.

    def sample(self,pids):
        now=time.monotonic()
        if now-self.previous<5:return
        self.previous=now;seen=set();queue=list(pids);rss=0;cpu=0
        while queue:
            pid=queue.pop()
            if pid in seen:continue
            seen.add(pid)
            try:
                stat=Path('/proc/%d/stat'%pid).read_text().rsplit(')',1)[1].split()
                cpu+=int(stat[11])+int(stat[12])
                status=Path('/proc/%d/status'%pid).read_text().splitlines()
                rss+=sum(int(line.split()[1]) for line in status if line.startswith('VmRSS:'))
                queue += [int(x) for x in Path('/proc/%d/task/%d/children'%(pid,pid)).read_text().split()]
            except (OSError,ValueError):continue
        memory={line.split(':')[0]:int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines() if line.startswith(('MemAvailable:','SwapFree:'))}
        self.samples.append(dict(monotonic_sec=now,tree_rss_kib=rss,cpu_seconds=cpu/os.sysconf('SC_CLK_TCK'),
            processes=len(seen),sampling_wall_sec=time.monotonic()-now,load1=os.getloadavg()[0],**memory))

    def save(self,path):Path(path).write_text(json.dumps(self.samples,indent=2))
