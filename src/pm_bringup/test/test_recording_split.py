"""topic分類・metadata保存・旧launch互換を、実センサ/モーターを起動せず確認する。"""
import ast
from pathlib import Path
from types import SimpleNamespace

import yaml
from pm_bringup.recording_metadata import atomic_json, copy_configuration, redact
from pm_bringup.record_trial import parameter_value


ROOT = Path(__file__).resolve().parents[1]


def test_core_console_diagnostics_are_opt_in_and_reversible():
    """センサを起動せずlaunch引数を評価し、詳細ログへの復帰経路を検査する。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from pm_bringup.stack_launch import create_launch_description
    context = LaunchContext()
    for action in create_launch_description(drive_default=False).entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['openvins_verbosity'] == 'WARNING'
    assert context.launch_configurations['vio_adapter_diagnostics'] == 'false'
    assert context.launch_configurations['vio_adapter_diagnostics_interval'] == '1.0'
    assert context.launch_configurations['openvins_imu_history_depth'] == '32'
    # 呼出元から指定した詳細診断値をdefaultで上書きしない。
    explicit = LaunchContext()
    explicit.launch_configurations.update(openvins_verbosity='DEBUG',
                                          vio_adapter_diagnostics='true',
                                          vio_adapter_diagnostics_interval='0.1',
                                          openvins_imu_history_depth='5')
    for action in create_launch_description(drive_default=False).entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(explicit)
    assert explicit.launch_configurations['openvins_verbosity'] == 'DEBUG'
    assert explicit.launch_configurations['vio_adapter_diagnostics_interval'] == '0.1'
    assert explicit.launch_configurations['openvins_imu_history_depth'] == '5'


def test_profiles_are_disjoint_and_cover_legacy_topics():
    """旧topicを落とさず上位追加とwheel昇格を確認する。"""
    profiles = yaml.safe_load((ROOT/'config/bag_profiles.yaml').read_text())
    mission, debug = set(profiles['mission']), set(profiles['debug'])
    assert len(mission) == len(profiles['mission'])
    assert len(debug) == len(profiles['debug'])
    assert not mission & debug
    tree = ast.parse((ROOT/'pm_bringup/stack_launch.py').read_text())
    old = next(ast.literal_eval(n.value) for n in ast.walk(tree) if isinstance(n,ast.Assign)
               and any(isinstance(t,ast.Name) and t.id=='bag_topics' for t in n.targets))
    assert set(old) <= mission | debug
    assert {'/parameter_events','/set_pose','/wheel/odometry','/motor_state','/tf_static'} <= mission
    assert '/ov_msckf/trackhist' in debug
    assert {'/oak/stereo/confidence/image_raw', '/oak/stereo/disparity/image_raw',
            '/oak/stereo/recording_snapshot', '/oak/diagnostics/confidence_frame',
            '/oak/diagnostics/disparity_frame'} <= mission


def test_metadata_redaction_and_config_references(tmp_path):
    """認証キーは伏せ、実効値の型と相対校正snapshotを保持する。"""
    values = {'camera':{'fx':100.0}, 'password':'do-not-store', 'nodes':['mapper']}
    assert redact(values)['password'] == '<redacted>'
    atomic_json(tmp_path/'snapshot.json', values)
    assert 'do-not-store' not in (tmp_path/'snapshot.json').read_text()
    config = tmp_path/'estimator.yaml'
    config.write_text('relative_config_imu: imu.yaml\n')
    (tmp_path/'imu.yaml').write_text('frequency: 100\n')
    index = copy_configuration([config],tmp_path/'saved')
    assert len(index)==2
    assert all('sha256' in e for e in index)
    assert (tmp_path/'saved'/index[0]['copy']).parent.joinpath('imu.yaml').is_file()


def test_foxy_parameter_value_types():
    """Foxyで高level parameter clientがなくても型を保って保存する。"""
    assert parameter_value(SimpleNamespace(type=2,integer_value=3)) == 3
    assert parameter_value(SimpleNamespace(type=7,integer_array_value=[2,3,5])) == [2,3,5]
    assert parameter_value(SimpleNamespace(type=0)) is None


def test_recorder_rejects_invalid_discovery_interval():
    """不正な探索周期はROS初期化やrecorder起動より前に拒否する。"""
    import pytest
    from pm_bringup.record_trial import main
    with pytest.raises(SystemExit) as error:
        main(['--profile', 'debug', '--output', '/tmp/not-created-by-this-test',
              '--workspace', '/tmp', '--external-workspace', '/tmp',
              '--polling-interval', '0'])
    assert error.value.code == 2


def test_bag_diagnostic_arguments_keep_existing_defaults():
    """探索周期100 ms、明示QoS指定なし、専用recorder OFFの既定を確認する。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from pm_bringup.bag_launch import create_bag_launch
    context = LaunchContext()
    for action in create_bag_launch('mission').entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['polling_interval_ms'] == '100'
    assert context.launch_configurations['qos_profile_overrides_path'] == ''
    assert context.launch_configurations['max_bag_size_bytes'] == '1000000000'
    assert context.launch_configurations['imu_sidecar'] == 'false'
    assert context.launch_configurations['bulk_sidecar'] == 'false'
    assert context.launch_configurations['max_cache_messages'] == '0'
    assert context.launch_configurations['writer_backend'] == 'standard'
    assert context.launch_configurations['writer_queue_bytes'] == str(64*1024*1024)
    assert context.launch_configurations['writer_queue_items'] == '8192'
    assert context.launch_configurations['mcap_preset_profile'] == 'default'


def test_oak_diagnostic_history_keeps_default_budget():
    """履歴容量だけを明示し、診断の既定OFFと150000 sampleを維持する。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from pm_bringup.stack_launch import create_launch_description
    context = LaunchContext()
    for action in create_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['oak_timing_diagnostics'] == 'false'
    assert context.launch_configurations['oak_timing_max_samples'] == '150000'


def test_async_queue_item_limit_must_be_positive():
    """item上限の無効化や負数を起動前に拒否する。"""
    import pytest
    from pm_bringup.record_trial import main
    with pytest.raises(SystemExit) as error:
        main(['--profile', 'mission', '--output', '/tmp/not_created_item_limit_test',
              '--workspace', '/tmp', '--external-workspace', '/tmp',
              '--writer-queue-items', '0'])
    assert error.value.code == 2


def test_mcap_preset_environment_is_child_local():
    """環境の原本は変更せず、既定モードでは前回実験の圧縮設定を継承しない。"""
    import pytest
    from pm_bringup.record_trial import recorder_environment
    original = {'PATH': '/example', 'PM_MCAP_PRESET_PROFILE': 'none'}
    assert recorder_environment('default', original) == {'PATH': '/example'}
    assert recorder_environment('zstd_fast_crc', original)['PM_MCAP_PRESET_PROFILE'] == 'zstd_fast_crc'
    assert original['PM_MCAP_PRESET_PROFILE'] == 'none'
    with pytest.raises(ValueError):
        recorder_environment('unknown', original)


def test_mcap_preset_rejects_other_storage_before_ros_start():
    """圧縮指定をSQLite等へ誤適用した場合は、ノード起動前に拒否する。"""
    import pytest
    from pm_bringup.record_trial import main
    with pytest.raises(SystemExit):
        main(['--profile', 'mission', '--output', '/tmp/unused-preset',
              '--workspace', '/tmp', '--external-workspace', '/tmp',
              '--storage', 'sqlite3', '--mcap-preset-profile', 'zstd_fast_crc'])


def test_sidecar_rejects_debug_profile():
    """専用IMU記録はmissionだけに許可し、ROS起動前に誤設定を拒否する。"""
    import pytest
    from pm_bringup.record_trial import main
    with pytest.raises(SystemExit):
        main(['--profile', 'debug', '--output', '/tmp/unused',
              '--workspace', '/tmp', '--external-workspace', '/tmp', '--imu-sidecar'])


def test_async_rejects_standard_cache_before_ros_start():
    """非同期FIFOと標準cacheの混同を、storage種別によらず起動前に拒否する。"""
    import pytest
    from pm_bringup.record_trial import main
    with pytest.raises(SystemExit):
        main(['--profile', 'mission', '--output', '/tmp/unused-async',
              '--workspace', '/tmp', '--external-workspace', '/tmp',
              '--writer-backend', 'async', '--storage', 'sqlite3', '--max-cache-size', '1'])


def test_bulk_partition_is_disjoint_complete_and_preserves_unknown_topics():
    from pm_bringup.record_trial import partition_bulk_topics
    profiles = yaml.safe_load((ROOT/'config/bag_profiles.yaml').read_text())
    topics = profiles['mission']+['/future_sensor']
    main, bulk = partition_bulk_topics(topics)
    assert not set(main) & set(bulk)
    assert set(main) | set(bulk) == set(topics)
    assert '/future_sensor' in main and '/wit/imu' in main and '/tf_static' in main
    assert '/oak/depth/image_raw' in bulk and '/ov_msckf/points_slam' in bulk


def test_bulk_mode_rejects_debug_and_duplicating_mode_before_ros_start():
    import pytest
    from pm_bringup.record_trial import main
    for profile, extra in [('debug', []), ('mission', ['--imu-sidecar'])]:
        with pytest.raises(SystemExit):
            main(['--profile', profile, '--output', '/tmp/unused-bulk',
                  '--workspace', '/tmp', '--external-workspace', '/tmp',
                  '--bulk-sidecar', *extra])


def test_bulk_qos_is_bounded_and_covers_exact_partition():
    """分離側だけを1.5秒にし、mainの設定や新しいtopicを暗黙に変更しない。"""
    from pm_bringup.record_trial import BULK_TOPICS
    qos = yaml.safe_load((ROOT/'config/bag_qos_bulk_buffered.yaml').read_text())
    assert set(qos) == BULK_TOPICS
    assert qos['/oak/stereo/left/image_raw']['depth'] == 30
    assert qos['/oak/depth/image_raw']['depth'] == 15
    assert all(value['history'] == 'keep_last' for value in qos.values())


def test_stop_signals_all_owned_recorders_and_late_registration():
    """全childへ一度だけ停止を送り、停止要求後に起動したchildも回収する。"""
    import signal
    import threading
    from pm_bringup.record_trial import BagControl
    class Child:
        def __init__(self):
            self.signals = []
        def poll(self):
            return None
        def send_signal(self, value):
            self.signals.append(value)
    fake = SimpleNamespace(children=[], _signal_lock=threading.RLock(),
                           _stop_requested=threading.Event(), _sigint_sent=False)
    first, second, late = Child(), Child(), Child()
    BagControl.add_child(fake, first)
    BagControl.add_child(fake, second)
    BagControl.request_stop(fake, publish=False)
    BagControl.request_stop(fake, publish=False)
    BagControl.add_child(fake, late)
    assert all(c.signals == [signal.SIGINT] for c in (first, second, late))


def test_sidecar_verification_requires_closed_metadata_and_info(tmp_path, monkeypatch):
    """bag infoだけ成功しても、metadata欠如や生存中childは成功扱いしない。"""
    import pm_bringup.record_trial as module
    child = SimpleNamespace(poll=lambda: 0, returncode=0)
    monkeypatch.setattr(module.subprocess, 'run', lambda *a, **k:
                        SimpleNamespace(returncode=0, stdout='fixture', stderr=''))
    monkeypatch.setattr(module, 'verify_message_counts', lambda path: {'verified': True})
    assert not module.verify_recorder(tmp_path, child)['verified']
    (tmp_path/'metadata.yaml').write_text('fixture')
    assert module.verify_recorder(tmp_path, child)['verified']
    assert not module.verify_recorder(tmp_path, SimpleNamespace(poll=lambda: None))['verified']


def test_sidecar_verification_reports_info_timeout(tmp_path, monkeypatch):
    """終了確認timeoutを記録し、GUIへ成功を返さない。"""
    import subprocess
    import pm_bringup.record_trial as module
    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired('ros2 bag info', 30)
    monkeypatch.setattr(module.subprocess, 'run', timeout)
    result = module.verify_recorder(tmp_path, SimpleNamespace(poll=lambda: 0, returncode=0))
    assert not result['verified'] and result['error']


def test_mission_buffered_qos_has_bounded_histories_and_static_durability():
    """大きい画像は約1秒分に抑え、遅延参加用の固定TFは従来通り保持する。"""
    qos = yaml.safe_load((ROOT/'config/bag_qos_mission_buffered.yaml').read_text())
    assert qos['/oak/stereo/left/image_raw']['depth'] == 20
    assert qos['/oak/depth/image_raw']['depth'] == 10
    assert qos['/wit/imu']['depth'] == 200
    assert qos['/tf_static']['durability'] == 'transient_local'
    assert qos['/tf_static']['history'] == 'keep_all'


def test_qos_selection_keeps_debug_and_explicit_rollback(tmp_path):
    """missionだけ既定履歴を変更し、debugと明示した旧QoSを維持する。"""
    from pm_bringup.record_trial import select_qos_path
    assert select_qos_path(tmp_path, 'mission', '').name == 'bag_qos_mission_buffered.yaml'
    assert select_qos_path(tmp_path, 'debug', '').name == 'bag_qos.yaml'
    rollback = tmp_path/'bag_qos.yaml'
    assert select_qos_path(tmp_path, 'mission', str(rollback)) == rollback


def test_recorder_rejects_negative_split_size():
    """0は分割なしとして許容するが、負数はrecorder起動前に拒否する。"""
    import pytest
    from pm_bringup.record_trial import main
    with pytest.raises(SystemExit) as error:
        main(['--profile', 'debug', '--output', '/tmp/not-created-by-this-test',
              '--workspace', '/tmp', '--external-workspace', '/tmp', '--max-bag-size', '-1'])
    assert error.value.code == 2


def test_recorder_rejects_negative_cache_size():
    """負のcache容量はROS初期化や既存bagへの接触前に拒否する。"""
    import pytest
    from pm_bringup.record_trial import main
    with pytest.raises(SystemExit) as error:
        main(['--profile', 'debug', '--output', '/tmp/unused-cache',
              '--workspace', '/tmp', '--external-workspace', '/tmp', '--max-cache-size', '-1'])
    assert error.value.code == 2


def test_recorder_rejects_nonzero_mcap_cache():
    """保存されない可能性のあるMCAP cacheを、ROSやchild起動前に拒否する。"""
    import pytest
    from pm_bringup.record_trial import main
    with pytest.raises(SystemExit) as error:
        main(['--profile', 'mission', '--output', '/tmp/unused-cache',
              '--workspace', '/tmp', '--external-workspace', '/tmp', '--max-cache-size', '1'])
    assert error.value.code == 2


def test_oak_snapshot_is_saved_beside_mission_provenance(tmp_path):
    """実センサなしで、cameraの実効JSON保存と異常schemaの拒否を確認する。"""
    import json
    from pm_bringup.record_trial import ParameterSnapshots
    fake = SimpleNamespace(output=tmp_path, errors={})
    snapshot = {'schema_version': 1, 'mx_id': 'synthetic', 'eeprom': {'version': 7},
                'initial_config': {'confidence': 240}, 'out_config': None}
    ParameterSnapshots.capture_oak_snapshot(fake, SimpleNamespace(data=json.dumps(snapshot)))
    assert json.loads((tmp_path/'oak_stereo_snapshot.json').read_text()) == snapshot
    ParameterSnapshots.capture_oak_snapshot(fake, SimpleNamespace(data='{}'))
    assert 'oak_stereo_snapshot' in fake.errors


def test_git_snapshot_includes_headers_and_excludes_private(tmp_path):
    """隔離した仮repoで外部include差分・未追跡source・秘密ファイル除外を確認する。"""
    import subprocess
    from pm_bringup.recording_metadata import git_info
    repo = tmp_path/'repo'
    repo.mkdir()
    def git(*args):
        subprocess.run(['git','-C',str(repo),*args],check=True,capture_output=True)
    git('init')
    (repo/'include').mkdir()
    header = repo/'include/test.h'
    header.write_text('int value = 1;\n')
    git('add','include/test.h')
    git('-c','user.name=Fixture','-c','user.email=fixture@example.invalid','commit','-m','fixture')
    header.write_text('int value = 3;\n')
    (repo/'module.py').write_text('value = 3\n')
    (repo/'private.yaml').write_text('password: synthetic-secret\n')
    (repo/'.env').write_text('PASSWORD=synthetic-secret\n')
    saved = tmp_path/'snapshot'
    result = git_info(repo,saved)
    assert result['head'] and result['recent_commits']
    assert 'int value = 3' in (saved/'tracked_changes.patch').read_text()
    assert result['untracked_copied'] == ['module.py']
    assert {'private.yaml','.env'} <= set(result['untracked_not_copied'])


def test_core_and_legacy_descriptions_keep_interfaces():
    """generateだけ行う。OpaqueFunction/Node/Timerの実行はしない。"""
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from pm_bringup.stack_launch import create_launch_description
    core = create_launch_description(False)
    legacy = create_launch_description(True)
    names = lambda desc: {a.name for a in desc.entities if isinstance(a,DeclareLaunchArgument)}
    assert names(core) == names(legacy)
    assert {'mapper_depth_subscription_queue_depth','localization_mode','use_vehicle_interface'} <= names(core)
    assert any(isinstance(a,OpaqueFunction) for a in core.entities)


def test_core_runtime_has_no_recorder_and_legacy_keeps_one():
    """検証callbackまで実行し、Node/Timer自体は実行せず記録processの分離を確認する。"""
    import uuid
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
    from pm_bringup.stack_launch import create_launch_description
    for legacy in (False,True):
        description = create_launch_description(legacy)
        context = LaunchContext()
        for action in description.entities:
            if isinstance(action,DeclareLaunchArgument):
                action.execute(context)
        context.launch_configurations['bag_name'] = 'unit_'+uuid.uuid4().hex
        # DDS環境適用もOpaqueFunctionなので、起動を行わない検証callbackを順に実行する。
        runtime = []
        for action in description.entities:
            if isinstance(action, OpaqueFunction):
                runtime.extend(action.execute(context) or [])
        assert any(type(a) is ExecuteProcess for a in runtime) == legacy
