"""２回再生版の条件隔離と、時刻/CDRを変えないhub合成を確認する。"""
from pathlib import Path
import yaml
from pm_evaluation.localization_batch_io import records
from pm_evaluation.localization_batch_plan import create_job
from pm_evaluation.localization_batch_twopass import build_hub,attach_hub,variant_topic
from pm_evaluation.localization_batch_twopass_parallel import shared_job
from pm_evaluation.localization_vio_replay import BagWriter


def test_hub_keeps_common_once_and_vo_variants_isolated(tmp_path):
    definitions={'std_msgs/msg/String':b'string data\n'}
    original=tmp_path/'original';writer=BagWriter(original,definitions)
    writer.write(100,'/wheel/odometry','std_msgs/msg/String',b'wheel')
    writer.write(110,'/ov_msckf/odomimu','std_msgs/msg/String',b'recorded')
    writer.close()
    variants={}
    for variant in ('online-cam-calib_on','online-cam-calib_off'):
        bag=tmp_path/variant;writer=BagWriter(bag,definitions)
        writer.write(110,'/ov_msckf/odomimu','std_msgs/msg/String',variant.encode());writer.close();variants[variant]=bag
    # validate_bagはFoxyのCLIも呼び、生成metadataまで確認する。
    hub=tmp_path/'hub';build_hub(original,original,variants,hub)
    topics=['/wheel/odometry',*map(variant_topic,variants),'/ov_msckf/odomimu']
    rows=list(records(hub,topics))
    assert len(rows)==3 and rows[0]==(100,'/wheel/odometry','std_msgs/msg/String',b'wheel')
    assert not any(row[1]=='/ov_msckf/odomimu' for row in rows)
    assert {row[1]:row[3] for row in rows[1:]}=={variant_topic(v):v.encode() for v in variants}
    assert all(row[0]==110 for row in rows[1:])


def test_all_conditions_share_sensors_but_not_vo_or_dynamic_tf(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    config=Path(get_package_share_directory('pm_config'))/'config'
    hub=tmp_path/'hub';jobs=[]
    for index,(variant,wheel) in enumerate([('online-cam-calib_on',False),('online-cam-calib_off',False),('online-cam-calib_on',True)]):
        folder=tmp_path/str(index)
        job=attach_hub(create_job(hub,folder,config,'separated_offroad',.1,{},wheel_only=wheel,duration=10),hub,variant)
        jobs.append((job,folder))
    plan=shared_job(jobs,tmp_path/'group')
    assert len(plan['branches'])==3
    assert '/ov_msckf/odomimu' not in plan['input_topics']
    for index,variant in enumerate(('online-cam-calib_on','online-cam-calib_off')):
        spec=next(s for s in plan['nodes'] if s['namespace']=='/evaluation/case_'+str(index) and s['executable']=='vio_odom_adapter_node')
        remaps=dict(spec['remaps'])
        assert remaps['/ov_msckf/odomimu']==plan['input_remaps'][variant_topic(variant)]
        assert remaps['tf']=='/evaluation/case_'+str(index)+'/tf'
        assert plan['expected_subscriptions'][variant_topic(variant)]==1
    assert len(plan['services'])==len(set(plan['services']))
    assert len(plan['outputs'])==len(set(plan['outputs']))
    assert plan['expected_subscriptions']['/wheel/odometry']>=3


def test_domain_partition_does_not_split_the_input_iterator_plan(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    config=Path(get_package_share_directory('pm_config'))/'config'
    jobs=[]
    for i in range(6):
        folder=tmp_path/str(i)
        job=attach_hub(create_job(tmp_path/'hub',folder,config,'separated_offroad',0.,{},wheel_only=True,duration=10),tmp_path/'hub','recorded')
        jobs.append((job,folder))
    plan=shared_job(jobs,tmp_path/'group',graphs_per_domain=5)
    assert len(plan['partitions'])==2
    assert {s['domain_offset'] for s in plan['nodes']}=={0,1}
    assert len(plan['branches'])==6
    assert sum(len(p['outputs']) for p in plan['partitions'])==len(plan['outputs'])
    assert sum(p['expected_subscriptions']['/wheel/odometry'] for p in plan['partitions'])==plan['expected_subscriptions']['/wheel/odometry']


def test_incomplete_storage_directories_are_visible_without_splitting_files(tmp_path):
    """metadata欠落を隠さず、分割storageは同じ試行として数える。"""
    from pm_evaluation.localization_batch_io import discover, discover_incomplete
    complete = tmp_path / 'complete'
    writer = BagWriter(complete, {'std_msgs/msg/String': b'string data\n'})
    writer.write(100, '/example', 'std_msgs/msg/String', b'data')
    writer.close()
    attached = complete / 'provenance'
    attached.mkdir()
    (attached / 'auxiliary.mcap').write_bytes(b'auxiliary')
    incomplete = tmp_path / 'incomplete'
    incomplete.mkdir()
    (incomplete / 'data_0.mcap').write_bytes(b'partial')
    (incomplete / 'data_1.mcap').write_bytes(b'partial')
    output = tmp_path / 'results'
    output.mkdir()
    (output / 'derived.db3').write_bytes(b'not input')
    assert discover(tmp_path, [output]) == [complete]
    assert discover_incomplete(tmp_path, [output]) == [incomplete]
    assert discover_incomplete(complete) == []


def test_missing_wheel_does_not_stop_next_bag(tmp_path, monkeypatch):
    """実際の必須入力検査で1件目が失敗しても、2件目の計画を最後まで作る。"""
    import json
    import sys
    import pytest
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage
    from pm_evaluation.cli import analyze_localization_batch_twopass as cli
    root = tmp_path / 'input'
    root.mkdir()
    for name in ('01_missing_wheel', '02_valid'):
        writer = BagWriter(root / name, {'std_msgs/msg/String': b'string data\n'})
        writer.write(100, '/example', 'std_msgs/msg/String', b'data')
        writer.close()
    # 最後にmetadata欠落の候補も置き、一覧から消えず失敗理由が保存されることを確認。
    incomplete = root / '03_incomplete'
    incomplete.mkdir()
    (incomplete / 'data_0.mcap').write_bytes(b'partial')
    visited = []

    def load(bag, *args):
        visited.append(bag.name)
        if bag.name == '01_missing_wheel':
            return {}
        return {'/wheel/odometry': [(100, Odometry())],
                '/wit/imu': [(100, Imu())], '/tf_static': [(100, TFMessage())]}

    monkeypatch.setattr(cli, 'load', load)
    output = tmp_path / 'output'
    monkeypatch.setattr(sys, 'argv', ['analyze_localization_batch_twopass', str(root),
        '--output-root', str(output), '--plan-only', '--stages', 'sources',
        '--vio-variants', 'recorded', '--magnetic', 'on', '--initial-yaw-deg', '0'])
    with pytest.raises(SystemExit) as error:
        cli.main()
    assert error.value.code == 1  # 全件を処理した後でも、失敗があれば終了codeへ残す。
    results = json.loads((output / 'summary.json').read_text())
    assert [r['status'] for r in results] == ['failed', 'planned', 'failed']
    assert '必要入力なし: /wheel/odometry' in results[0]['reason']
    wheel_jobs = [j for j in results[1]['jobs'] if j['stage'] == 'sources']
    assert wheel_jobs and all(j['status'] == 'planned' for j in wheel_jobs)
    assert 'metadata.yamlなし' in results[2]['reason']
    assert visited == ['01_missing_wheel', '02_valid']
    assert all(r['replays'] == {'vo': 0, 'ekf': 0} for r in results)


def test_index_write_error_does_not_stop_following_bag(tmp_path, monkeypatch):
    """索引保存失敗が例外処理から漏れて後続試行を止めないことを確認する。"""
    import json
    import sys
    import pytest
    from pm_evaluation.cli import analyze_localization_batch_twopass as cli
    root = tmp_path / 'input'
    root.mkdir()
    for name in ('01', '02'):
        writer = BagWriter(root / name, {'std_msgs/msg/String': b'string data\n'})
        writer.write(100, '/example', 'std_msgs/msg/String', b'data')
        writer.close()
    original = cli.update_index
    calls = []

    def update_index(folder, results):
        calls.append(len(results))
        if len(calls) == 1:
            raise OSError('索引の一時書込エラー')
        original(folder, results)

    monkeypatch.setattr(cli, 'update_index', update_index)
    monkeypatch.setattr(cli, 'load', lambda *args: {})
    output = tmp_path / 'output'
    monkeypatch.setattr(sys, 'argv', ['batch', str(root), '--output-root', str(output),
                                    '--plan-only', '--stages', 'recorded'])
    with pytest.raises(SystemExit) as error:
        cli.main()
    assert error.value.code == 1
    assert calls == [1, 2]
    results = json.loads((output / 'summary.json').read_text())
    assert [r['status'] for r in results] == ['failed', 'planned']
    assert results[0]['index_error'] == '索引の一時書込エラー'
