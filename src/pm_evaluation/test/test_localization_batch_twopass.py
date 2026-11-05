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
