"""OpenVINS外部校正A/B用の設定複製・raw再生成・低帯域bag合成。"""
from collections import Counter
import heapq
import json
from pathlib import Path
import re
import shutil
import subprocess

import yaml
from mcap.reader import make_reader
from mcap.writer import Writer
from pm_evaluation.localization_batch_io import metadata, records, TOPICS


SENSOR_TOPICS=('/oak/stereo/left/image_raw','/oak/stereo/right/image_raw','/oak/imu/data')


class BagWriter:
    """元schemaを再使用してMCAPとFoxy互換metadataを完成時に確定する。"""
    def __init__(self,folder,schemas):
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=False)
        self.stream=(self.folder/'data_0.mcap').open('wb');self.writer=Writer(self.stream)
        self.writer.start(profile='ros2');self.schemas=schemas;self.channels={};self.counts=Counter()
        self.first=None;self.last=None

    def write(self,ns,topic,typename,raw):
        if topic not in self.channels:
            definition=self.schemas.get(typename)
            if definition is None:raise ValueError('MCAP schema不明: '+typename)
            sid=self.writer.register_schema(typename,'ros2msg',definition)
            cid=self.writer.register_channel(topic,'cdr',sid)
            self.channels[topic]=(cid,typename)
        self.writer.add_message(self.channels[topic][0],int(ns),raw,int(ns))
        self.counts[topic]+=1
        self.first=int(ns) if self.first is None else min(self.first,int(ns))
        self.last=int(ns) if self.last is None else max(self.last,int(ns))

    def close(self):
        self.writer.finish();self.stream.close()
        info=dict(version=4,compression_format='',compression_mode='',storage_identifier='mcap',relative_file_paths=['data_0.mcap'],
                  duration={'nanoseconds':0 if self.first is None else self.last-self.first},
                  starting_time={'nanoseconds_since_epoch':self.first or 0},
                  message_count=sum(self.counts.values()),topics_with_message_count=[])
        for topic,count in self.counts.items():
            info['topics_with_message_count'].append(dict(message_count=count,topic_metadata=dict(
                name=topic,type=self.channels[topic][1],serialization_format='cdr',offered_qos_profiles='')))
        (self.folder/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':info},sort_keys=False))


def schemas(bag):
    """記録bagの完全ROS message定義を得る。別topicの同じ型も利用する。"""
    result={};info,paths=metadata(bag)
    if info['storage_identifier']=='mcap':
        for path in paths:
            with path.open('rb') as stream:
                summary=make_reader(stream).get_summary()
                if summary:
                    result.update({s.name:s.data for s in summary.schemas.values()})
    return result


def configuration_source(bag,explicit=None):
    """保存された初期校正の出所を読み取り専用で選ぶ。"""
    bag=Path(bag)
    if explicit:source=Path(explicit).resolve()
    else:
        index=bag/'provenance/configuration_index.json';candidates=[]
        if index.exists():
            for entry in json.loads(index.read_text()):
                if Path(entry.get('source','')).name.startswith('estimator_config'):
                    p=bag/'provenance/configuration'/entry['copy']
                    if p.is_file():candidates.append(p)
        if len(candidates)!=1:raise ValueError('保存OpenVINS設定を一意に選べない。--vio-configを指定')
        source=candidates[0]
    return source


def configuration_files(bag,explicit=None):
    """VO cache用に、初期設定と参照される校正ファイルを列挙する。"""
    source=configuration_source(bag,explicit)
    paths=[source]
    for relative in re.findall(r'^relative_config_\w+:\s*[\"\']?([^\"\'\s#]+)',source.read_text(),re.M):
        p=(source.parent/relative).resolve()
        if not p.is_file():raise ValueError('初期校正ファイル不足: '+str(p))
        paths.append(p)
    return paths


def configuration(bag,destination,variant,explicit=None):
    """OpenCV形式YAMLの初期校正を複製し、外部校正flagだけを変更する。"""
    bag=Path(bag);destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    source=configuration_source(bag,explicit)
    text=source.read_text()
    # 外部ファイルは同じ初期値を全条件で使う。存在しない場合は現在の別HW値を流用しない。
    paths=re.findall(r'^relative_config_\w+:\s*[\"\']?([^\"\'\s#]+)',text,re.M)
    for relative in paths:
        path=(source.parent/relative).resolve()
        if not path.is_file():raise ValueError('初期校正ファイル不足: '+str(path))
        target=destination/relative
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    # 旧名は既存の呼出しとの互換用。新しいCLI・保存先では明示的な名称を使う。
    on=variant in ('online-cam-calib_on','online_on')
    if not re.search(r'^calib_cam_extrinsics:\s*(true|false)',text,re.M):
        raise ValueError('calib_cam_extrinsics設定がない')
    text=re.sub(r'^(calib_cam_extrinsics:\s*)(true|false)',lambda m:m[1]+('true' if on else 'false'),text,flags=re.M)
    target=destination/'estimator_config.yaml';target.write_text(text)
    flags={k:v=='true' for k,v in re.findall(r'^(calib_\w+):\s*(true|false)',text,re.M)}
    (destination/'conditions.json').write_text(json.dumps(dict(source=str(source),variant=variant,
         flags=flags,changed_only='calib_cam_extrinsics',
         note='OFFは起動時のcamera–IMU外部パラメータを固定。内参・時差・IMU校正flagは維持'),ensure_ascii=False,indent=2))
    return target


def merge_input(original,raw_bag,output,input_records=None):
    """画像は再保存せず、センサと新raw VIOを記録時刻順に合成する。"""
    allowed=[t for t in TOPICS if t not in ('/ov_msckf/odomimu','/vio/odometry','/visual/odometry',
                     '/ov_msckf/diagnostics/health')]
    definitions=schemas(original);definitions.update(schemas(raw_bag))
    writer=BagWriter(output,definitions)
    try:
        for ns,topic,name,raw in heapq.merge(records(original if input_records is None else input_records,allowed),
                records(raw_bag,['/ov_msckf/odomimu','/ov_msckf/diagnostics/health']),key=lambda r:r[0]):
            writer.write(ns,topic,name,raw)
    finally:writer.close()
    validate_bag(output)


def validate_bag(folder):
    """Foxy CLIはmetadata読込失敗でもexit=0の場合があるため本文も検査する。"""
    result=subprocess.run(['ros2','bag','info',str(folder)],capture_output=True,text=True)
    if result.returncode or '[ERROR]' in result.stdout+result.stderr:
        raise RuntimeError('bag info検証失敗: '+result.stdout+result.stderr)
