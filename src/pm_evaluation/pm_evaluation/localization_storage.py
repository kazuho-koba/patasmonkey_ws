"""解析bagの可逆圧縮と、分離照合後の共有bag破棄を扱う。"""
import hashlib
from collections import Counter
from mcap.reader import make_reader
import heapq
import json
import os
import re
from pathlib import Path
import shutil
import struct

import yaml

from pm_evaluation.localization_batch_io import records,metadata
from pm_evaluation.localization_vio_replay import BagWriter,schemas,validate_bag


def recorder_environment(base=None, compression=None):
    """Foxy対応MCAP overlayの既存presetをrecorder childだけへ設定する。

    transportのfile圧縮ではなくMCAP内chunk圧縮を使い、拡張子やreaderを変えない。
    markerがなければ未対応pluginで黙って非圧縮記録せず、起動前に失敗させる。
    """
    env=dict(os.environ if base is None else base)
    compression=compression or env.get('PM_EVALUATION_MCAP_COMPRESSION','zstd')
    if compression not in ('zstd','none'):raise ValueError('未知のMCAP圧縮指定: '+compression)
    if compression=='zstd':
        from ament_index_python.packages import get_package_share_directory
        marker=Path(get_package_share_directory('rosbag2_storage_mcap'))/'pm_foxy_preset_support.yaml'
        if not marker.is_file():raise RuntimeError('ZSTD preset対応のrosbag2_storage_mcap overlayが必要。source/buildを確認するか--mcap-compression noneを指定')
        capability=yaml.safe_load(marker.read_text())
        if capability.get('environment')!='PM_MCAP_PRESET_PROFILE' or 'zstd_fast_crc' not in capability.get('profiles',[]):
            raise RuntimeError('MCAP overlayのzstd_fast_crc対応を確認できない')
    env['PM_MCAP_PRESET_PROFILE']='zstd_fast_crc' if compression=='zstd' else 'none'
    return env


def _update(digest,row):
    """topic改名後の記録ns・型・CDR・順序を、曖昧な連結を避けてhashする。"""
    ns,topic,typename,raw=row
    digest.update(struct.pack('<q',int(ns)))
    for value in (topic.encode(),typename.encode(),bytes(raw)):
        digest.update(struct.pack('<Q',len(value)));digest.update(value)


def split_and_verify(group_bags,branches,report):
    """全分離先をbag infoと全record hashで照合。１件でも不一致なら削除しない。

    入力は今回workerで完成した共有bagだけ。戻り値のverifiedがtrueになって初めて
    cleanupへ進む。分離先には従来どおりZSTDのPython BagWriterを使う。
    """
    definitions={}
    for bag in group_bags:validate_bag(bag);definitions.update(schemas(bag))
    writers={};destinations={};expected={};counts={};types={}
    try:
        for branch in branches:
            target=Path(branch['folder'])/'output_bag';key=str(target)
            if key in writers:raise ValueError('分離先が重複: '+key)
            writer=BagWriter(target,definitions);writers[key]=writer;expected[key]=hashlib.sha256();counts[key]=0;types[key]=set()
            for source,topic in branch['outputs'].items():
                if source in destinations:raise ValueError('共有出力topicの分離先が重複: '+source)
                destinations[source]=(key,writer,topic)
        for bag in group_bags:
            info,_=metadata(bag)
            recorded={x['topic_metadata']['name'] for x in info['topics_with_message_count'] if x['message_count']}
            if recorded-set(destinations):raise ValueError('共有bagに分離計画外のtopicがある。削除せず保持する')
        streams=[records(bag,list(destinations)) for bag in group_bags]
        for ns,source,typename,raw in heapq.merge(*streams,key=lambda row:row[0]):
            key,writer,topic=destinations[source]
            writer.write(ns,topic,typename,raw);_update(expected[key],(ns,topic,typename,raw))
            counts[key]+=1;types[key].add(topic)
    finally:
        for writer in writers.values():writer.close()
    results=[]
    for key in writers:
        validate_bag(Path(key));actual=hashlib.sha256();count=0
        for row in records(Path(key),sorted(types[key])):_update(actual,row);count+=1
        entry=dict(output=key,messages=count,expected_messages=counts[key],sha256=actual.hexdigest(),
                   expected_sha256=expected[key].hexdigest(),verified=count==counts[key] and actual.digest()==expected[key].digest())
        results.append(entry)
    data=dict(verified=all(r['verified'] for r in results),bags=results)
    Path(report).write_text(json.dumps(data,indent=2))
    if not data['verified']:raise RuntimeError('分離前後の時刻/CDR/順序が不一致。共有bagを保持: '+str(report))
    return data


def cleanup_shared(group,group_bags,verification,keep=False):
    """今回のgroup直下の共有bagだけを削除。既存解析や分離先・元bagへは触れない。"""
    group=Path(group).resolve()
    if not verification.get('verified') or not verification.get('bags'):raise ValueError('照合成功なしの共有bag削除は許可しない')
    paths=[];bytes_before=0;compression=Counter()
    for bag in group_bags:
        bag=Path(bag)
        # symlinkを辿る削除・metadataの範囲外part参照を拒否し、対象をgroup直下へ限定。
        if (bag.name!='output_bag' and not re.fullmatch(r'output_bag_\d+',bag.name)) or bag.is_symlink() or bag.resolve().parent!=group or bag.resolve() in [Path(r['output']).resolve() for r in verification['bags']]:
            raise ValueError('共有bag削除対象がgroup直下でない/分離先と同一: '+str(bag))
        _,parts=metadata(bag)
        for part in parts:
            with part.open('rb') as stream:summary=make_reader(stream).get_summary()
            if summary:
                compression.update(index.compression or 'none' for index in summary.chunk_indexes)
        bytes_before+=sum(p.stat().st_size for p in bag.rglob('*') if p.is_file())
        paths.append(bag)
    if not keep:
        for bag in paths:shutil.rmtree(bag)
    (group/'shared_storage.json').write_text(json.dumps(dict(verified=True,kept=keep,
        shared_bytes=bytes_before,chunk_compression=dict(compression),reclaimed_bytes=0 if keep else bytes_before,paths=[str(p) for p in paths]),indent=2))
