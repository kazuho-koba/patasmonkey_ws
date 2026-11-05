"""圧縮後の内容保存と、全分離先の照合成功時だけ共有bagを消すことを検証する。"""
import json
from pathlib import Path
import pytest
from mcap.reader import make_reader
from pm_evaluation.localization_vio_replay import BagWriter
from pm_evaluation.localization_storage import split_and_verify,cleanup_shared,recorder_environment


def shared(tmp_path):
    group=tmp_path/'group';group.mkdir();source=group/'output_bag_0'
    writer=BagWriter(source,{'std_msgs/msg/String':b'string data\n'})
    for i in range(50):
        writer.write(100+i,'/case0/local','std_msgs/msg/String',b'x'*1024)
        writer.write(100+i,'/case1/local','std_msgs/msg/String',b'y'*1024)
    writer.close()
    branches=[dict(folder=str(tmp_path/str(i)),outputs={'/case%d/local'%i:'/odometry/local'}) for i in range(2)]
    return group,source,branches


def test_split_zstd_and_cleanup_preserves_final_bags(tmp_path):
    group,source,branches=shared(tmp_path)
    result=split_and_verify([source],branches,group/'split_verification.json')
    assert result['verified'] and [b['messages'] for b in result['bags']]==[50,50]
    for branch in branches:
        file=Path(branch['folder'])/'output_bag/data_0.mcap'
        with file.open('rb') as stream:summary=make_reader(stream).get_summary()
        assert summary.chunk_indexes and all(c.compression=='zstd' for c in summary.chunk_indexes)
    cleanup_shared(group,[source],result)
    assert not source.exists()
    assert all((Path(b['folder'])/'output_bag/metadata.yaml').exists() for b in branches)
    saved=json.loads((group/'shared_storage.json').read_text())
    assert saved['reclaimed_bytes']>0 and saved['chunk_compression']['zstd']>0


def test_keep_shared_and_refuse_unverified_delete(tmp_path):
    group,source,branches=shared(tmp_path)
    with pytest.raises(ValueError):cleanup_shared(group,[source],dict(verified=False))
    assert source.exists()
    result=split_and_verify([source],branches,group/'verification.json')
    cleanup_shared(group,[source],result,keep=True)
    assert source.exists()
    assert json.loads((group/'shared_storage.json').read_text())['reclaimed_bytes']==0


def test_hash_mismatch_keeps_shared_bag(tmp_path,monkeypatch):
    from pm_evaluation import localization_storage as storage
    group,source,branches=shared(tmp_path);original=storage.records
    def changed(bag,topics):
        for ns,topic,name,raw in original(bag,topics):
            if Path(bag).name=='output_bag':raw=b'tampered'
            yield ns,topic,name,raw
    monkeypatch.setattr(storage,'records',changed)
    with pytest.raises(RuntimeError,match='不一致'):split_and_verify([source],branches,group/'verification.json')
    assert source.exists()
    assert not json.loads((group/'verification.json').read_text())['verified']


def test_cleanup_rejects_source_outside_group(tmp_path):
    group,source,branches=shared(tmp_path)
    result=split_and_verify([source],branches,group/'verification.json')
    with pytest.raises(ValueError):cleanup_shared(group,[tmp_path/'outside'],result)
    assert source.exists()


def test_recorder_preset_is_child_only_and_crc_preserving(monkeypatch):
    monkeypatch.delenv('PM_MCAP_PRESET_PROFILE',raising=False)
    env=recorder_environment({'ROS_DOMAIN_ID':'192'},'zstd')
    assert env['PM_MCAP_PRESET_PROFILE']=='zstd_fast_crc'
    assert env['ROS_DOMAIN_ID']=='192'
    import os
    assert 'PM_MCAP_PRESET_PROFILE' not in os.environ
    assert recorder_environment({},'none')['PM_MCAP_PRESET_PROFILE']=='none'
