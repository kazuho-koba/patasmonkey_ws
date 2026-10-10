"""実機へ接続せず、現行ソースの指令計算と制限状態の更新を検証する。

ROS/ODriveの初期化を避けるため対象メソッドをASTで抽出する。
motorの代替オブジェクトは指令を保存するだけで、通信や駆動を行わない。
"""

import ast
import math
from pathlib import Path
import random
from types import SimpleNamespace

import pytest


class Twist:
    """指令計算で参照するTwistの2成分を保持する。"""
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0)
        self.angular = SimpleNamespace(z=0.0)


class Stamp:
    """制御周期を整数ナノ秒で再現する。"""
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return Stamp(self.nanoseconds - other.nanoseconds)


def make_control(alpha=1.0, yaw_cap=1.0):
    """製品ソースを抽出し、YAMLと同じ寸法とメモリ内の出力先を設定する。"""
    source = Path(__file__).resolve().parents[1] / 'pm_control/vehicle_interface_node.py'
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'VehicleInterfaceNode')
    names = {'map_joy_twist_to_vehicle_twist', 'clamp', 'reset_motion_limits',
             'apply_motion_limits', 'motor_control', 'joy_state_callback', 'joy_speed_limit',
             'command_selector', 'cmd_vel_callback_joy'}
    body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    tree = ast.Module(body=[ast.ClassDef(name='Control', bases=[], keywords=[],
                                       body=body, decorator_list=[])], type_ignores=[])
    env = {'math': math, 'Twist': Twist}
    exec(compile(ast.fix_missing_locations(tree), str(source), 'exec'), env)
    a = env['Control']()
    a.max_linear_speed, a.min_turn_radius = 1.0, 0.7
    a.joy_normal_max_linear_speed = 1.0
    a.joy_turbo_max_linear_speed = 2.0
    a.autonomous_max_linear_speed = 1.0
    a.joy_turbo_button = 1
    a._joy_turbo_active = False
    a._joy_state_received_at = None
    a.max_yaw_rate, a.max_yaw_accel = yaw_cap, alpha
    a.wheel_radius, a.tread_width, a.gear_ratio = 0.1016, 0.36, 10.0
    a.max_whl_rps = 4.0
    a.left_motor_sign, a.right_motor_sign = 1.0, -1.0
    a.joy_max_turn_angle_deg, a.joy_side_stop_angle_deg = 45.0, 80.0
    a.now_ns = 0
    # Joyの鮮度判定も同じ制御時刻で再現し、実時間への依存をなくす。
    env['time'] = SimpleNamespace(monotonic=lambda: a.now_ns * 1e-9)
    a.get_clock = lambda: SimpleNamespace(now=lambda: Stamp(a.now_ns))
    a.left_motor = SimpleNamespace(set_velocity=lambda value: setattr(a, 'left_sent', value))
    a.right_motor = SimpleNamespace(set_velocity=lambda value: setattr(a, 'right_sent', value))
    a.sim_cmd_vel_pub = SimpleNamespace(publish=lambda msg: setattr(a, 'sim', msg))
    a.odrive_connected = True
    a.reset_motion_limits()
    return a


def drive(a, v, w, dt=0.04, speed_limit=None):
    """時刻を進めて速度指令を処理し、最終車輪指令の等価Twistを返す。"""
    a.now_ns += int(dt * 1e9)
    msg = Twist()
    msg.linear.x, msg.angular.z = v, w
    a.motor_control(msg, speed_limit)
    circumference = 2 * math.pi * a.wheel_radius
    left = a.left_cmd_rps / a.gear_ratio * circumference
    right = a.right_cmd_rps / a.gear_ratio * circumference
    return (left + right) / 2, (right - left) / a.tread_width


def settle(a, v, w):
    """初期値に依存しない定常指令まで計算を進める。"""
    for _ in range(150):
        result = drive(a, v, w)
    return result


def test_reducing_turn_never_amplifies_speed():
    """元の9.6倍増幅を引き起こす入力履歴で共通上限を守る。"""
    a = make_control()
    settle(a, 1.0, 1.0 / 0.7)
    for _ in range(100):
        v, w = drive(a, 1.0, 0.1)
        assert 0.0 < v <= 1.0 + 1e-12
        assert abs(w) <= 1.0 + 1e-12


def test_straight_return_and_turn_reversal_do_not_stop():
    """前後方向が変わらない操舵変更ではゼロ速度を挟まない。"""
    for target in (0.0, -1.0 / 0.7):
        a = make_control()
        settle(a, 1.0, 1.0 / 0.7)
        for _ in range(60):
            previous = a.prev_yaw_rate_cmd
            v, w = drive(a, 1.0, target)
            assert v > 0.6
            assert abs(w - previous) <= 0.04 + 1e-12


@pytest.mark.parametrize('alpha', [0.5, 1.0, 2.0])
def test_linear_and_yaw_slew_use_yaw_accel(alpha):
    """yaw加速度パラメータが速度移行率とyaw移行率の両方に反映される。"""
    a = make_control(alpha=alpha)
    settle(a, 1.0, 0.0)
    previous_v = a.prev_linear_speed_cmd
    v, w = drive(a, 1.0, 1.0 / 0.7)
    assert previous_v - v == pytest.approx(alpha * 0.7 * 0.04)
    assert w == pytest.approx(alpha * 0.04)


@pytest.mark.parametrize('yaw_cap', [0.25, 0.5, 1.0, 2.0])
def test_steady_curvature_and_adaptive_speed(yaw_cap):
    """yaw上限を変更すると速度が適応し、最大旋回の定常半径を維持する。"""
    a = make_control(yaw_cap=yaw_cap)
    v, w = settle(a, 1.0, 1.0 / 0.7)
    expected = 1.0 / (1.0 + (1.0 / (0.7 * yaw_cap)) ** 4) ** 0.25
    assert v == pytest.approx(expected)
    assert v / w == pytest.approx(0.7)
    assert abs(w) <= yaw_cap


def test_stop_timeout_and_restart():
    """停止とタイムアウトで履歴をゼロにし、再開も設定した変化率を守る。"""
    a = make_control()
    settle(a, 1.0, 1.0)
    assert drive(a, 0.0, 1.0) == (0.0, 0.0)
    assert drive(a, 1.0, 0.0)[0] == pytest.approx(0.028)
    a.motor_control(None)
    assert a.prev_linear_speed_cmd == a.prev_yaw_rate_cmd == 0.0
    assert drive(a, 1.0, 0.0)[0] == pytest.approx(0.028)


def test_forward_reverse_crosses_zero():
    """前後反転は停止を優先し、逆方向への再開をゼロ履歴から行う。"""
    a = make_control()
    settle(a, 1.0, 0.0)
    assert drive(a, -1.0, 0.0) == (0.0, 0.0)
    assert drive(a, -1.0, 0.0)[0] == pytest.approx(-0.028)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_invalid_input_stops(bad):
    """非有限入力をモータ指令へ渡さない。"""
    a = make_control()
    assert drive(a, bad, 0.0) == (0.0, 0.0)
    assert drive(a, 1.0, bad) == (0.0, 0.0)


def test_wheel_limit_state_and_sim_are_consistent():
    """厳しい車輪上限でも保持状態とsim出力が最終モータ指令に一致する。"""
    a = make_control(alpha=0.0)
    a.max_whl_rps = 0.5
    v, w = drive(a, 1.0, 1.0)
    assert max(abs(a.left_cmd_rps), abs(a.right_cmd_rps)) <= 5.0 + 1e-12
    assert a.prev_linear_speed_cmd == pytest.approx(v)
    assert a.prev_yaw_rate_cmd == pytest.approx(w)
    assert a.sim.linear.x == pytest.approx(v)
    assert a.sim.angular.z == pytest.approx(w)


def test_randomized_transitions_preserve_hard_limits():
    """混在する手動相当・自律相当指令で速度/yaw/半径の絶対制約を確認する。"""
    a = make_control()
    rng = random.Random(2718)
    for _ in range(1500):
        v, w = drive(a, rng.uniform(-4, 4), rng.uniform(-6, 6))
        assert abs(v) <= 1.0 + 1e-12
        assert abs(w) <= 1.0 + 1e-12
        assert abs(w) <= abs(v) / 0.7 + 1e-12
        assert max(abs(a.left_cmd_rps), abs(a.right_cmd_rps)) <= 40 + 1e-12


def test_joystick_sign_stop_angle_and_turbo_cap():
    """元の操作方向・真横停止を保ち、turboや独立軸の最大値も速度上限で丸める。"""
    a = make_control()
    for x, y in [(1, 1), (1, -1), (-1, 1), (-1, -1), (2, 0)]:
        msg = Twist()
        msg.linear.x, msg.angular.z = x, y
        mapped = a.map_joy_twist_to_vehicle_twist(msg)
        assert abs(mapped.linear.x) <= 1.0
        assert mapped.linear.x * x > 0.0
        if y:
            assert mapped.angular.z * (x * y) > 0.0
    msg = Twist()
    msg.linear.x = math.cos(math.radians(81))
    msg.angular.z = math.sin(math.radians(81))
    mapped = a.map_joy_twist_to_vehicle_twist(msg)
    assert mapped.linear.x == mapped.angular.z == 0.0


def test_clock_pause_backward_and_long_delay():
    """時計の停止・巻き戻りは速度を増やさず、長い遅延でも最大0.1秒分だけ進める。"""
    a = make_control()
    assert drive(a, 1.0, 0.0, dt=0.0) == (0.0, 0.0)
    assert drive(a, 1.0, 0.0, dt=-0.04) == (0.0, 0.0)
    assert drive(a, 1.0, 0.0, dt=2.0)[0] == pytest.approx(0.07)


@pytest.mark.parametrize('turbo, magnitude, expected', [
    (False, 1.0, 1.0), (False, 2.0, 1.0), (True, 2.0, 2.0)])
def test_turbo_mode_has_separate_straight_speed(turbo, magnitude, expected):
    """直進の通常/Turboが別上限へ到達し、通常入力の過大値を丸める。"""
    a = make_control()
    a.max_linear_speed = 2.0
    msg = Twist()
    msg.linear.x = magnitude
    for _ in range(120):
        a.joy_state_callback(SimpleNamespace(buttons=[0, int(turbo)]))
        limit = a.joy_speed_limit()
        mapped = a.map_joy_twist_to_vehicle_twist(msg, limit)
        v, _ = drive(a, mapped.linear.x, mapped.angular.z, speed_limit=limit)
    assert v == pytest.approx(expected)


def test_normal_diagonal_is_not_turbo():
    """Twistのノルムが1を超えても明示的なボタンなしで通常上限を超えない。"""
    a = make_control()
    a.max_linear_speed = 2.0
    msg = Twist()
    msg.linear.x, msg.angular.z = 1.0, 0.4
    mapped = a.map_joy_twist_to_vehicle_twist(msg, a.joy_speed_limit())
    assert mapped.linear.x == 1.0


def test_turbo_release_stale_and_missing_button_fall_back():
    """Turbo解除・古いJoy・欠損/無効ボタンで通常上限へ戻す。"""
    a = make_control()
    a.max_linear_speed = 2.0
    assert a.joy_speed_limit() == 1.0
    a.joy_state_callback(SimpleNamespace(buttons=[0, 1]))
    assert a.joy_speed_limit() == 2.0
    a.now_ns += 300_000_000
    assert a.joy_speed_limit() == 1.0
    for buttons in ([0, 0], []):
        a.joy_state_callback(SimpleNamespace(buttons=buttons))
        assert a.joy_speed_limit() == 1.0
    a.joy_turbo_button = -1
    a.joy_state_callback(SimpleNamespace(buttons=[1, 1]))
    assert a.joy_speed_limit() == 1.0


def test_turbo_release_caps_previous_fast_command():
    """解除直後に、速度ランプや残ったTurboのTwistより通常上限を優先する。"""
    a = make_control()
    a.max_linear_speed = 2.0
    for _ in range(100):
        drive(a, 2.0, 0.0, speed_limit=2.0)
    assert a.prev_linear_speed_cmd == 2.0
    v, _ = drive(a, 2.0, 0.0, speed_limit=1.0)
    assert v == 1.0


def test_autonomous_speed_remains_normal_when_turbo_ceiling_increases():
    """共通上限2 m/sでも自律指令の選択では従来の1 m/sを維持する。"""
    a = make_control()
    a.max_linear_speed = 2.0
    a._emergency_stop_active = False
    a._drive_armed = True
    a.last_cmd_vel_joy = None
    a.last_cmd_vel_time = Stamp(0)
    a.last_cmd_vel = Twist()
    a.last_cmd_vel.linear.x = 2.0
    captured = []
    a.motor_control = lambda cmd, limit: captured.append((cmd, limit))
    a.command_selector()
    assert captured == [(a.last_cmd_vel, 1.0)]


def test_selector_rechecks_turbo_state_for_cached_twist():
    """最新Joyで変換をやり直し、同じTwistの保持中にもTurbo解除を反映する。"""
    a = make_control()
    a.max_linear_speed = 2.0
    a._emergency_stop_active = False
    a._drive_armed = True
    a.last_cmd_vel = None
    msg = Twist()
    msg.linear.x = 2.0
    a.cmd_vel_callback_joy(msg)
    captured = []
    a.motor_control = lambda cmd, limit: captured.append((cmd.linear.x, limit))
    a.joy_state_callback(SimpleNamespace(buttons=[0, 1]))
    a.command_selector()
    a.joy_state_callback(SimpleNamespace(buttons=[0, 0]))
    a.command_selector()
    assert captured == [(2.0, 2.0), (1.0, 1.0)]
