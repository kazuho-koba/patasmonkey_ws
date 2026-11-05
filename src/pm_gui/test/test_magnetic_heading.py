"""ROS・GUI・実機を使わず、方位の符号と傾斜補償を検証する。"""
import math
import pytest
from pm_gui.magnetic_heading import magnetic_bearing, cardinal


@pytest.mark.parametrize('field,bearing', [((1,0,0),0), ((0,1,0),90),
                                         ((-1,0,0),180), ((0,-1,0),270)])
def test_cardinals(field,bearing):
    assert magnetic_bearing(field,0,0) == pytest.approx(bearing)


def test_tilt_and_bias():
    # 水平化前のベクトルをRx^-1 Ry^-1で生成。北成分1、下向き成分2。
    r,p = .3,.4
    x = math.cos(p)*1-math.sin(p)*(-2)
    z = math.sin(p)*1+math.cos(p)*(-2)
    field = (x,math.sin(r)*z,math.cos(r)*z)
    assert magnetic_bearing(field,r,p) == pytest.approx(0,abs=1e-9)
    assert magnetic_bearing((2,1,1),0,0,bias=(1,1,1),offset_deg=10) == pytest.approx(10)
    assert cardinal(90)=='E'


@pytest.mark.parametrize('field,pitch', [((0,0,1),0), ((math.nan,1,1),0),
                                      ((1,0,0),math.pi/2)])
def test_invalid(field,pitch):
    with pytest.raises(ValueError):
        magnetic_bearing(field,0,pitch)


@pytest.mark.parametrize('previous', [False, True])
def test_gui_and_initializer_use_same_selected_calibration(tmp_path, previous):
    """GUIのYAML読込から方位計算までを確認し、coreとの角度規約を照合する。"""
    from pathlib import Path
    import yaml
    from pm_gui.operator_console import load_config
    from pm_localization.heading_initializer_node import magnetic_yaw
    root = Path(__file__).resolve().parents[2]
    source = root / 'pm_config/config/hwt905_mag_calibration.yaml'
    calibration = yaml.safe_load(source.read_text())
    if previous:
        lines = [line.strip()[2:] for line in source.read_text().splitlines()
                 if line.strip().startswith(('# bias:', '# scale:', '# offset_deg:'))]
        calibration['wit_mag_corrector']['ros__parameters'].update(yaml.safe_load('\n'.join(lines)))
    path = tmp_path / 'hwt905_mag_calibration.yaml'
    path.write_text(yaml.safe_dump(calibration))
    document = yaml.safe_load((root / 'pm_gui/config/gui.yaml').read_text())
    # 同じ共通YAMLでコメントを切り替えた状態を再現し、実機やwindowは起動しない。
    document['operator_console']['ros__parameters']['compass']['calibration_file'] = str(path)
    gui_path = tmp_path / 'gui.yaml'
    gui_path.write_text(yaml.safe_dump(document))
    cfg = load_config(str(gui_path))['compass']
    expected = yaml.safe_load(path.read_text())['wit_mag_corrector']['ros__parameters']
    assert cfg['config_error'] == ''
    assert cfg['offset_deg'] == expected['offset_deg']
    raw = (1000., -300., 1200.)
    bearing = magnetic_bearing(raw, .1, -.1, cfg['bias'], cfg['scale'],
                               cfg['offset_deg'], cfg['declination_deg'])
    field = [(raw[i] - cfg['bias'][i]) * cfg['scale'][i] for i in range(3)]
    yaw = magnetic_yaw(field, .1, -.1, cfg['offset_deg'], cfg['declination_deg'])
    difference = yaw - math.radians(90. - bearing)
    assert math.atan2(math.sin(difference), math.cos(difference)) == pytest.approx(0.)
