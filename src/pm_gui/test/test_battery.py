"""実機を接続せず、停止電圧と正のIbus積算の分岐を確認する。"""

import math

import pytest

from pm_gui.battery import BatteryEstimator


@pytest.fixture
def sample(monkeypatch):
    # データ時刻と受信時計を明示的に進め、sleepなしで実際の秒/Ah換算を確認する。
    clock = [100.0]
    monkeypatch.setattr('pm_gui.battery.time.monotonic', lambda: clock[0])
    estimator = BatteryEstimator()

    def receive(seconds, voltage=19.0, current=0.0, stationary=False):
        clock[0] = 100.0+seconds
        return estimator.update(voltage, ibus_a=current, stationary=stationary,
                                sample_time=1000.0+seconds)

    return estimator, receive, clock


def test_positive_ibus_uses_total_parallel_capacity(sample):
    estimator, receive, _ = sample
    initial = receive(0, stationary=True)
    assert initial['percent'] == 50.0
    assert initial['capacity_ah'] == 12.0
    # 12Aを360秒: 1.2Ah消費、12Ahに対して10%減少。低い走行電圧で再換算しない。
    for t in range(1, 361):
        result = receive(t, voltage=16.8, current=12.0)
    assert result['percent'] == pytest.approx(40.0)
    assert result['consumed_ah'] == pytest.approx(1.2)
    assert result['anchor_voltage_v'] == 19.0
    assert result['level'] == 'LOW'
    assert result['estimate_mode'] == 'COULOMB_COUNTING'


def test_negative_ibus_never_adds_charge(sample):
    _, receive, _ = sample
    receive(0, stationary=True)
    consumed = receive(1, current=12.0)
    for t in range(2, 61):
        result = receive(t, voltage=20.0, current=-30.0)
    assert result['percent'] == consumed['percent']
    assert result['consumed_ah'] == consumed['consumed_ah']


def test_stop_rebases_only_after_five_seconds(sample):
    _, receive, _ = sample
    receive(0, stationary=True)
    before_stop = receive(1, voltage=16.0, current=12.0)
    for t in range(2, 7):
        pending = receive(t, voltage=20.0, stationary=True)
        assert pending['estimate_mode'] == 'WAIT_SETTLE'
        assert pending['percent'] == before_stop['percent']
    stopped = receive(7, voltage=20.0, stationary=True)
    assert stopped['estimate_mode'] == 'REST_ESTIMATE'
    assert stopped['percent'] == 80.0
    assert stopped['consumed_ah'] == 0.0
    assert stopped['anchor_voltage_v'] == 20.0


def test_initial_moving_sample_waits_for_stop_baseline(sample):
    _, receive, _ = sample
    result = receive(0, voltage=17.0, current=20.0)
    assert result['percent'] is None
    assert result['estimate_mode'] == 'WAIT_BASELINE'
    stopped = receive(1, stationary=True)
    assert stopped['percent'] == 50.0


@pytest.mark.parametrize('current', [None, math.nan, math.inf])
def test_invalid_moving_current_requires_new_baseline(sample, current):
    _, receive, _ = sample
    receive(0, stationary=True)
    invalid = receive(1, current=current)
    assert invalid['percent'] is None
    assert invalid['estimate_mode'] == 'CURRENT_INVALID'
    recovered = receive(2, current=12.0)
    assert recovered['percent'] is None
    assert recovered['estimate_mode'] == 'WAIT_BASELINE'


@pytest.mark.parametrize('voltage', [0.0, -1.0, math.nan, math.inf])
def test_invalid_voltage_is_not_empty_battery(sample, voltage):
    _, receive, _ = sample
    receive(0, stationary=True)
    result = receive(1, voltage=voltage)
    assert result['percent'] is None
    assert result['level'] == 'INVALID'


def test_gap_and_time_reversal_do_not_integrate_unknown_interval(sample):
    _, receive, _ = sample
    receive(0, stationary=True)
    assert receive(4, current=60.0)['percent'] is None
    receive(5, stationary=True)
    assert receive(3, current=60.0)['percent'] is None


def test_motorstate_time_controls_integral_at_fast_replay(sample):
    estimator, _, clock = sample
    estimator.update(19.0, ibus_a=0.0, stationary=True, sample_time=1000.0)
    # 再生が10倍速でも、600秒のデータは12A*600/3600=2Ahとして積算する。
    for t in range(1, 601):
        clock[0] += 0.1
        result = estimator.update(17.0, ibus_a=12.0, sample_time=1000.0+t)
    assert result['consumed_ah'] == pytest.approx(2.0)
    assert result['percent'] == pytest.approx(50.0-100.0*2.0/12.0)


def test_receiver_gap_invalidates_even_if_message_time_is_contiguous(sample):
    estimator, _, clock = sample
    estimator.update(19.0, ibus_a=0.0, stationary=True, sample_time=1000.0)
    clock[0] += 4.0
    result = estimator.update(19.0, ibus_a=20.0, sample_time=1001.0)
    assert result['percent'] is None


def test_capacity_is_configurable_and_percent_is_bounded(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr('pm_gui.battery.time.monotonic', lambda: clock[0])
    estimator = BatteryEstimator({'capacity_ah_per_pack': 0.1, 'parallel_packs': 1})
    estimator.update(19.0, stationary=True)
    for t in range(1, 101):
        clock[0] = t
        result = estimator.update(19.0, ibus_a=100.0)
    assert result['percent'] == 0.0
    assert result['capacity_ah'] == 0.1


@pytest.mark.parametrize('config', [
    {'capacity_ah_per_pack': 0}, {'capacity_ah_per_pack': math.nan},
    {'parallel_packs': 0}, {'parallel_packs': 1.5}, {'parallel_packs': math.inf},
])
def test_invalid_capacity_is_rejected(config):
    with pytest.raises(ValueError):
        BatteryEstimator(config)


def test_backend_reads_new_motorstate_ibus(sample):
    """実際のROS message定義を使い、callbackが共通bus値を渡すことを確認する。"""
    from types import SimpleNamespace
    from pm_msgs.msg import MotorState
    from pm_gui.ros_backend import RosBackend

    estimator, _, clock = sample
    stored = {}
    backend = SimpleNamespace(
        _battery_estimator=estimator,
        _battery_stationary_cmd_rps=0.001,
        _battery_stationary_vel_rps=0.1,
        _store=lambda key, value: stored.update({key: value}),
    )
    message = MotorState()
    message.stamp.sec = 1000
    message.vbus_voltage = 19.0
    message.ibus_a = 0.0
    RosBackend._motor_state_callback(backend, message)
    assert stored['battery']['percent'] == 50.0
    message.stamp.sec = 1001
    message.ibus_a = 12.0
    message.left_cmd_rps = message.right_cmd_rps = 1.0
    message.left_vel_rps = message.right_vel_rps = 1.0
    clock[0] += 1.0
    RosBackend._motor_state_callback(backend, message)
    assert stored['battery']['percent'] == pytest.approx(50.0-100.0/3600.0)
    assert stored['battery']['current_a'] == 12.0


def test_qt_panel_displays_coulomb_mode_and_current(sample, monkeypatch):
    """画面/実機を使わず、Qtのlabel更新と電池描画を確認する。"""
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    from PyQt5.QtWidgets import QApplication
    from pm_gui.panels import BatteryPanel

    _, receive, _ = sample
    receive(0, stationary=True)
    result = receive(1, current=12.0)
    application = QApplication.instance() or QApplication([])
    panel = BatteryPanel({})
    panel.resize(160, 240)
    panel.refresh({'telemetry': {'battery': {'value': result, 'age': 0.0}}})
    panel.show()
    application.processEvents()
    assert panel.status.text() == '走行中 / 電流積算'
    assert 'Ibus 12.00 A' in panel.status.toolTip()
    assert not panel.grab().isNull()
    panel.refresh({'telemetry': {'battery': {'value': result, 'age': 4.0}}})
    assert panel.status.text() == 'STALE'
    assert panel.gauge.percent is None
    panel.close()
