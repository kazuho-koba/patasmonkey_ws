"""共通校正YAMLのbias/scaleと初期化offsetの整合、および旧値の復元を検証する。"""
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from pm_bringup.magnetic_calibration import prepare_magnetic_calibration

CONFIG = Path(__file__).resolve().parents[2] / 'pm_config/config'


@pytest.mark.parametrize('previous,offset', [
    (False, -7.6554087986239985), (True, -12.9701),
])
def test_shared_yaml_and_commented_previous_values(tmp_path, previous, offset):
    """共通YAMLの現行値と、コメント内の旧値のどちらも初期化へ渡せる。"""
    source = CONFIG / 'hwt905_mag_calibration.yaml'
    document = yaml.safe_load(source.read_text())
    if previous:
        # コメントの3行を有効化する操作を再現し、旧値の保存内容も検査する。
        lines = [line.strip()[2:] for line in source.read_text().splitlines()
                 if line.strip().startswith(('# bias:', '# scale:', '# offset_deg:'))]
        restored = yaml.safe_load('\n'.join(lines))
        assert set(restored) == {'bias', 'scale', 'offset_deg'}
        document['wit_mag_corrector']['ros__parameters'].update(restored)
    path = tmp_path / 'hwt905_mag_calibration.yaml'
    path.write_text(yaml.safe_dump(document))
    context = LaunchContext()
    selected = prepare_magnetic_calibration(context, PathJoinSubstitution([str(path)]))
    values = document['wit_mag_corrector']['ros__parameters']
    assert selected == path.resolve()
    assert ParameterValue(LaunchConfiguration('_mag_offset_deg'), value_type=float).evaluate(context) == offset
    assert float(context.launch_configurations['_mag_declination_deg']) == values['declination_deg']
    assert values['calibrated'] is False


@pytest.mark.parametrize('key,value', [
    ('bias', [0, 0]), ('bias', [0, float('nan'), 0]),
    ('scale', [1, 0, 1]), ('offset_deg', float('inf')),
])
def test_invalid_calibration_stops_before_nodes_are_started(tmp_path, key, value):
    values = yaml.safe_load((CONFIG / 'hwt905_mag_calibration.yaml').read_text())
    values['wit_mag_corrector']['ros__parameters'][key] = value
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(values))
    context = LaunchContext()
    with pytest.raises(ValueError, match='磁気校正YAML'):
        prepare_magnetic_calibration(context, PathJoinSubstitution([str(path)]))
    assert '_mag_offset_deg' not in context.launch_configurations


def test_core_uses_selected_file_before_returning_runtime_actions(tmp_path, monkeypatch):
    """OpaqueFunctionだけを評価し、実機nodeを起動せず共通設定とmanifest準備を確認する。"""
    monkeypatch.setenv('HOME', str(tmp_path))
    from pm_bringup.stack_launch import create_launch_description
    description = create_launch_description(drive_default=False)
    context = LaunchContext()
    context.launch_configurations.update(
        bag_name='magnetic_profile_test',
    )
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    # 共通stack末尾が起動前検査。先頭のDDS環境設定もOpaqueFunctionなので除く。
    opaque = [a for a in description.entities if isinstance(a, OpaqueFunction)][-1]
    actions = opaque.execute(context)
    assert actions
    assert float(context.launch_configurations['_mag_offset_deg']) == -7.6554087986239985
    assert 'mag_calibration_config' not in context.launch_configurations
