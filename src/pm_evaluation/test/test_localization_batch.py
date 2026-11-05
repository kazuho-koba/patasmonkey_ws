"""分割bag境界・探索除外・履歴代用・観測無効化の回帰試験。"""
from pathlib import Path
import sqlite3
import yaml
import pytest
from pm_evaluation.localization_batch_io import discover, metadata, records, fingerprint, MODES
from pm_evaluation.localization_batch_plan import create_job


def bag(tmp_path,name='trial',storage='sqlite3'):
    root=tmp_path/name;root.mkdir(parents=True)
    paths=['record_2.db3','record_10.db3']
    for path in paths:
        connection=sqlite3.connect(str(root/path))
        connection.executescript('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT); CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB);')
        connection.execute('INSERT INTO topics VALUES(1,?,?)',('/fix','sensor_msgs/msg/NavSatFix'))
        connection.execute('INSERT INTO messages VALUES(1,1,?,?)',(paths.index(path)+1,b'data'))
        connection.commit();connection.close()
    (root/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{'storage_identifier':storage,'relative_file_paths':paths}}))
    return root


def test_discovery_uses_metadata_not_split_files_and_excludes_output(tmp_path):
    first=bag(tmp_path);derived=bag(tmp_path/'results','derived')
    assert discover(tmp_path,[tmp_path/'results'])==[first]
    assert discover(first)==[first]
    assert [p.name for p in metadata(first)[1]]==['record_2.db3','record_10.db3']
    assert [r[0] for r in records(first,['/fix'])]==[1,2]


def test_metadata_rejects_missing_or_escaping_part(tmp_path):
    root=bag(tmp_path)
    (root/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{'relative_file_paths':['../secret'],'storage_identifier':'sqlite3'}}))
    with pytest.raises(ValueError):metadata(root)


def test_cache_key_changes_with_config_and_input_metadata(tmp_path):
    root=bag(tmp_path);code=tmp_path/'code.py';code.write_text('a')
    original=fingerprint(root,{'magnetic':'on'},[code])
    assert original!=fingerprint(root,{'magnetic':'off'},[code])
    code.write_text('b');assert original!=fingerprint(root,{'magnetic':'on'},[code])


@pytest.mark.parametrize('mode',MODES)
def test_all_graphs_are_non_actuating_and_use_new_adapter(tmp_path,mode):
    from ament_index_python.packages import get_package_share_directory
    config=Path(get_package_share_directory('pm_config'))/'config'
    job=create_job(tmp_path,tmp_path/mode,config,mode,.4,{},False,False)
    assert all(s['package'] in ('robot_localization','pm_localization') for s in job['nodes'])
    assert any(s['executable']=='vio_odom_adapter_node' for s in job['nodes'])
    assert not any('cmd_vel' in topic for topic in job['input_topics'])
    local=yaml.safe_load((tmp_path/mode/'batch_local.yaml').read_text())['batch_local']['ros__parameters']
    assert local['initial_state'][5]==.4 and local['use_sim_time']


def test_wheel_only_disables_projected_vio_and_marks_historical_substitutions(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    config=Path(get_package_share_directory('pm_config'))/'config'
    job=create_job(tmp_path,tmp_path/'job',config,'separated_projected',0.,{},True,True)
    local=yaml.safe_load((tmp_path/'job/batch_local.yaml').read_text())['batch_local']['ros__parameters']
    assert not any(local['twist1_config'])
    assert '/ov_msckf/odomimu' not in job['input_topics']
    assert not any(s['executable']=='vio_odom_adapter_node' for s in job['nodes'])
    assert job['substitutions']


def test_mcap_writer_roundtrip_and_metadata_boundary(tmp_path):
    from pm_evaluation.localization_vio_replay import BagWriter
    root=tmp_path/'raw'
    writer=BagWriter(root,{'std_msgs/msg/String':b'string data\n'})
    writer.write(100,'/example','std_msgs/msg/String',b'cdr')
    writer.write(120,'/example','std_msgs/msg/String',b'cdr2')
    writer.close()
    assert discover(root)==[root]
    assert [(x[0],x[3]) for x in records(root,['/example'])]==[(100,b'cdr'),(120,b'cdr2')]
    info,_=metadata(root)
    assert info['message_count']==2 and info['duration']['nanoseconds']==20
    assert info['compression_format']=='' and info['compression_mode']==''


@pytest.mark.parametrize('variant,enabled', [('online-cam-calib_on',True),('online-cam-calib_off',False),('online_on',True),('online_off',False)])
def test_vio_flag_change_preserves_other_online_calibrations(tmp_path,variant,enabled):
    from pm_evaluation.localization_vio_replay import configuration
    config=tmp_path/'source';config.mkdir()
    (config/'imu.yaml').write_text('imu0: {}')
    source=config/'estimator.yaml'
    source.write_text('%YAML:1.0\ncalib_cam_extrinsics: true # comment\ncalib_cam_intrinsics: true\nrelative_config_imu: "imu.yaml"\n')
    out=configuration(tmp_path,tmp_path/'off',variant,source)
    assert ('calib_cam_extrinsics: '+str(enabled).lower()) in out.read_text()
    assert 'calib_cam_intrinsics: true' in out.read_text()
    assert (out.parent/'imu.yaml').exists()


def test_clean_removes_untyped_empty_arrays():
    from pm_evaluation.localization_batch_plan import clean
    assert clean({'datum':[],'other':None,'frequency':10.,'flags':[False]*15})=={'frequency':10.,'flags':[False]*15}


def test_quality_rejects_negative_variance_instead_of_accepting_existing_plot():
    from nav_msgs.msg import Odometry
    from pm_evaluation.cli.analyze_localization_batch import output_quality
    msg=Odometry();msg.twist.covariance[7]=-.001
    result=output_quality({'/odometry/local':[(0,msg)]})
    assert result['numerically_invalid']


def test_quality_separates_physical_drift_warning_from_invalid_covariance():
    from nav_msgs.msg import Odometry
    from pm_evaluation.cli.analyze_localization_batch import output_quality
    msg=Odometry();msg.twist.twist.linear.y=4.
    result=output_quality({'/odometry/local':[(0,msg)]})
    assert not result['numerically_invalid'] and result['speed_warning']


def test_shared_graph_isolates_dynamic_tf_services_and_adapter_reference(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    from pm_evaluation.localization_batch_parallel import shared_job
    config=Path(get_package_share_directory('pm_config'))/'config'
    jobs=[]
    for mode in ('separated_offroad','ekf_3d_no_accel'):
        folder=tmp_path/mode
        jobs.append((create_job(tmp_path,folder,config,mode,.2,{},duration=10),folder))
    plan=shared_job(jobs,tmp_path/'group')
    assert len(plan['branches'])==2
    for i in range(2):
        prefix='/evaluation/case_'+str(i)
        for spec in [s for s in plan['nodes'] if s['namespace']==prefix]:
            remaps=dict(spec['remaps'])
            assert remaps['/tf']==prefix+'/tf'
            assert remaps['/tf_static']=='/evaluation/input/tf_static'
            assert remaps['tf_static']=='/evaluation/input/tf_static'
            assert remaps['tf']==prefix+'/tf'
            assert remaps['/odometry/local']==prefix+'/odometry/local'
            assert remaps['/wit/imu']=='/evaluation/input/wit/imu'
            assert '/clock' not in remaps
            parameters=yaml.safe_load(Path(spec['parameters']).read_text())
            assert prefix+'/'+spec['name'] in parameters
    assert plan['expected_subscriptions']['/ov_msckf/odomimu']==2
    assert plan['expected_subscriptions']['/tf_static']==8
    assert len(set(plan['services']))==4
    assert len(set(plan['outputs']))==len(plan['outputs'])


def test_shared_replay_rejects_incompatible_time_ranges(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    from pm_evaluation.localization_batch_parallel import shared_job
    config=Path(get_package_share_directory('pm_config'))/'config'
    a=create_job(tmp_path,tmp_path/'a',config,'legacy',0.,{},duration=10)
    b=create_job(tmp_path,tmp_path/'b',config,'separated_offroad',0.,{},duration=20)
    with pytest.raises(ValueError):shared_job([(a,tmp_path/'a'),(b,tmp_path/'b')],tmp_path/'group')


def test_lowband_extract_preserves_payload_order_and_timestamps(tmp_path):
    from pm_evaluation.localization_batch_cache import extract_lowband,record_digest
    original=bag(tmp_path)
    extracted,report=extract_lowband(original,tmp_path/'cache')
    assert report['status']=='extracted_verified'
    assert record_digest(original)==record_digest(extracted)
    assert list(records(original,['/fix']))==list(records(extracted,['/fix']))
    _,again=extract_lowband(original,tmp_path/'cache')
    assert again['status']=='cached'


def test_lowband_corruption_is_not_reused(tmp_path):
    from pm_evaluation.localization_batch_cache import extract_lowband,record_digest
    original=bag(tmp_path);extracted,_=extract_lowband(original,tmp_path/'cache')
    connection=sqlite3.connect(str(extracted/'data.db3'))
    connection.execute('UPDATE messages SET data=? WHERE id=1',(b'changed',));connection.commit();connection.close()
    restored,report=extract_lowband(original,tmp_path/'cache')
    assert report['status']=='extracted_verified'
    assert record_digest(restored)==record_digest(original)


def test_lowband_rejects_reordered_split_timestamps(tmp_path):
    from pm_evaluation.localization_batch_cache import extract_lowband
    original=bag(tmp_path)
    connection=sqlite3.connect(str(original/'record_2.db3'))
    connection.execute('UPDATE messages SET timestamp=3');connection.commit();connection.close()
    with pytest.raises(ValueError,match='内容/時刻/順序'):
        extract_lowband(original,tmp_path/'cache')


def test_ekf_key_uses_effective_parameters_not_vio_label(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    from pm_evaluation.localization_batch_cache import ekf_key
    config=Path(get_package_share_directory('pm_config'))/'config'
    first=create_job(tmp_path,tmp_path/'a',config,'separated_offroad',.2,{},wheel_only=True)
    second=create_job(tmp_path,tmp_path/'b',config,'separated_offroad',.2,{},wheel_only=True)
    first['vio_variant']='online-cam-calib_on';second['vio_variant']='online-cam-calib_off'
    code=tmp_path/'worker.py';code.write_text('implementation')
    key=ekf_key(first,tmp_path/'a',{'sha256':'same'},[code],1)
    assert key==ekf_key(second,tmp_path/'b',{'sha256':'same'},[code],1)
    second['duration']=1.
    assert key!=ekf_key(second,tmp_path/'b',{'sha256':'same'},[code],1)
    assert key!=ekf_key(first,tmp_path/'a',{'sha256':'different'},[code],1)


def test_vio_cache_changes_only_with_vio_dependencies(tmp_path):
    from pm_evaluation.localization_batch_cache import vio_key
    settings=tmp_path/'estimator.yaml';settings.write_text('calib: true')
    code=tmp_path/'worker.py';code.write_text('run')
    render=tmp_path/'plot.py';render.write_text('old')
    key=vio_key('bag','online-cam-calib_on',1.,None,[settings],[code])
    render.write_text('new')
    assert key==vio_key('bag','online-cam-calib_on',1.,None,[settings],[code])
    assert key!=vio_key('bag','online-cam-calib_off',1.,None,[settings],[code])
    settings.write_text('calib: false')
    assert key!=vio_key('bag','online-cam-calib_on',1.,None,[settings],[code])


def test_stage_dependencies_exclude_plot_from_numerical_cache():
    from pm_evaluation.localization_batch_cache import stage_code_files
    stages=stage_code_files()
    assert all(p.name not in ('analyze_localization_batch.py','localization_batch_plot.py') for p in stages['vo']+stages['ekf'])


def test_result_marker_requires_matching_storage(tmp_path):
    import json
    from pm_evaluation.localization_batch_cache import result_ready,source_identity
    output=bag(tmp_path,'output_bag')
    (tmp_path/'complete.json').write_text(json.dumps({'status':'complete','storage':source_identity(output)}))
    assert result_ready(tmp_path)
    connection=sqlite3.connect(str(output/'record_2.db3'));connection.execute('UPDATE messages SET timestamp=9');connection.commit();connection.close()
    assert not result_ready(tmp_path)


def test_plot_notes_separate_missing_early_end_and_diagnostic():
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus
    from pm_evaluation.localization_batch_plot import figure_notes
    def row(sec):
        m=Odometry();m.header.stamp.sec=sec;return (sec*1000000000,m)
    health=DiagnosticArray();status=DiagnosticStatus()
    status.level=DiagnosticStatus.WARN;status.message='initializing';health.status=[status]
    data={'/vio/odometry':[row(1),row(1),row(1),row(2),row(2),row(3),row(7)],'/odometry/local':[row(12)],
          '/ov_msckf/diagnostics/health':[(0,health)]}
    notes=figure_notes(data,('/vio/odometry','/missing'),('/odometry/local',))
    assert any('INPUT NOT OBSERVED: /missing' in n for n in notes)
    assert any('INPUT ENDS EARLY: /vio/odometry' in n for n in notes)
    assert any('INPUT GAP: /vio/odometry' in n for n in notes)
    assert any('DIAGNOSTIC:' in n and 'initializing' in n for n in notes)


def test_draw_missing_series_saves_annotations_and_figures(tmp_path):
    import json
    import numpy as np
    from pm_evaluation.localization_batch_plot import draw
    draw(tmp_path,[],{'vio':np.empty((0,13))},None,'Unavailable', ['VO REPLAY FAILED'])
    notes=json.loads((tmp_path/'plot_annotations.json').read_text())
    assert 'VO REPLAY FAILED' in notes
    assert any('PLOT MISSING: vio' in n for n in notes)
    assert (tmp_path/'xy_full.png').is_file() and (tmp_path/'z.png').is_file()


def test_analysis_directory_uses_minute_names_and_reuses_identity(tmp_path):
    from pm_evaluation.localization_batch_cache import analysis_directory
    first=analysis_directory(tmp_path,'condition-a','20261103_2045')
    assert first.name=='20261103_2045'
    # 日時が変わっても同条件は再利用し、別条件と強制再実行は別directoryになる。
    assert analysis_directory(tmp_path,'condition-a','20261103_2046')==first
    other=analysis_directory(tmp_path,'condition-b','20261103_2045')
    assert other.name=='20261103_2045_02'
    forced=analysis_directory(tmp_path,'condition-a','20261103_2045',reuse=False)
    assert forced.name=='20261103_2045_03'
    assert analysis_directory(tmp_path,'condition-a','20261103_2047')==forced


def test_analysis_directory_preserves_legacy_and_ignores_invalid_marker(tmp_path):
    from pm_evaluation.localization_batch_cache import analysis_directory
    old=tmp_path/'8526422b1020fef2';old.mkdir();(old/'complete.json').write_text('{}')
    bad=tmp_path/'20261103_2044';bad.mkdir();(bad/'analysis_identity.json').write_text('partial')
    new=analysis_directory(tmp_path,'condition','20261103_2045')
    assert new.name=='20261103_2045' and old.exists() and bad.exists()


def test_analysis_directory_retries_failure_without_overwriting(tmp_path):
    from pm_evaluation.localization_batch_cache import analysis_directory
    ready=lambda p:(p/'success').exists()
    failed=analysis_directory(tmp_path,'same','20261103_2045',ready=ready)
    (failed/'worker.log').write_text('VO initialization failed')
    retry=analysis_directory(tmp_path,'same','20261103_2050',ready=ready)
    assert retry!=failed and retry.name=='20261103_2050'
    assert (failed/'worker.log').read_text()=='VO initialization failed'
    (retry/'success').touch()
    assert analysis_directory(tmp_path,'same','20261103_2055',ready=ready)==retry


def test_analysis_timestamp_is_jst_even_in_utc_container():
    from datetime import timedelta
    from pm_evaluation.localization_batch_cache import analysis_timestamp
    assert analysis_timestamp().utcoffset()==timedelta(hours=9)


def test_gnss_view_limits_include_every_point_and_stay_tight():
    from pm_evaluation.localization_batch_plot import gnss_view_limits
    gps=[(0,0.,0.,0.,'UNKNOWN'),(1,40.,-15.,0.,'UNKNOWN')]
    assert gnss_view_limits(gps)==[-2.,42.,-17.,2.]
    gps.append((2,1000.,0.,0.,'UNKNOWN'))
    assert gnss_view_limits(gps)[1]>1000.


def test_gnss_stationary_view_has_nonzero_extent():
    from pm_evaluation.localization_batch_plot import gnss_view_limits
    assert gnss_view_limits([(0,2.,3.,0.,'UNKNOWN')])==[1.,3.,2.,4.]
