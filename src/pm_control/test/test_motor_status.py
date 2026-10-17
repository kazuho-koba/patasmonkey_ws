"""ROS・USBを開かず、所有ノードによる実状態診断のON/OFFと失敗を検証する。"""
import ast
import json
from pathlib import Path
from types import SimpleNamespace


def make_node(enabled, broken=False):
    """対象関数だけを読み込み、パラメータ・軸・通知先をメモリ上で代替する。"""
    path = Path(__file__).resolve().parents[1] / 'pm_control/vehicle_interface_node.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                  and n.name == 'publish_vehicle_status')
    tree = ast.Module(body=[method], type_ignores=[])
    scope = {'json': json, 'String': SimpleNamespace}
    exec(compile(ast.fix_missing_locations(tree), str(path), 'exec'), scope)
    output = []
    motor = SimpleNamespace(axis=SimpleNamespace(current_state=8, error=0,
                                                 controller=SimpleNamespace(input_vel=0.0)))
    node = SimpleNamespace(odrive_connected=True, _drive_armed=True,
                           _emergency_stop_active=False, left_motor=None if broken else motor,
                           right_motor=motor, get_parameter=lambda _: SimpleNamespace(value=enabled),
                           _status_pub=SimpleNamespace(publish=lambda m: output.append(json.loads(m.data))))
    scope['publish_vehicle_status'](node)
    return output[0]


def test_default_does_not_read_axes():
    """OFFでは壊れた軸でも参照せず、既存通知の内容を保つ。"""
    assert make_node(False, broken=True) == {'connected': True, 'armed': True, 'emergency_stop': False}


def test_enabled_reports_actual_state():
    """許可フラグだけでなく左右の実状態と入力ゼロを通知する。"""
    assert make_node(True)['axes'] == [{'current_state': 8, 'error': 0, 'input_vel': 0.0}] * 2


def test_read_error_is_explicit():
    """読出し失敗時は成功値を作らず、エラーとして通知する。"""
    assert 'motor_state_read_error' in make_node(True, broken=True)
