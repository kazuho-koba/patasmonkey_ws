"""topic分類・metadata保存・旧launch互換を、実センサ/モーターを起動せず確認する。"""
import ast
from pathlib import Path
from types import SimpleNamespace

import yaml
from pm_bringup.recording_metadata import atomic_json, copy_configuration, redact
from pm_bringup.record_trial import parameter_value


ROOT = Path(__file__).resolve().parents[1]


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
        opaque = next(a for a in description.entities if isinstance(a,OpaqueFunction))
        runtime = opaque.execute(context)
        assert any(type(a) is ExecuteProcess for a in runtime) == legacy
