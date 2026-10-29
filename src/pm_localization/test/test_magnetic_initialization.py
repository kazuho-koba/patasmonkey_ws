"""磁気方位変換と円平均の回帰試験。"""
import math
from types import SimpleNamespace

import pytest
from pm_localization.heading_initializer_node import (
    HeadingInitializerNode, circular_mean, magnetic_yaw, wrap_pi)


def test_heading_conventions_and_offset():
    # 磁北ベクトルが車体+xなら北向き。x前/y左の東向きでは磁場は車体+y。
    assert magnetic_yaw((1., 0., 0.), 0., 0., 0., 0., .1) == pytest.approx(math.pi/2)
    assert magnetic_yaw((0., 1., 0.), 0., 0., 0., 0., .1) == pytest.approx(0.)
    assert magnetic_yaw((1., 0., 0.), 0., 0., 30., 0., .1) == pytest.approx(math.pi/3)
    with pytest.raises(ValueError):
        magnetic_yaw((0., 0., 1.), 0., 0., 0., 0., .1)


def test_circular_average_crosses_pi():
    mean, std = circular_mean([math.radians(179), math.radians(-179)])
    assert abs(abs(mean)-math.pi) < 1e-9
    assert std == pytest.approx(math.radians(1), rel=1e-3)
    assert abs(wrap_pi(mean - math.pi)) < 1e-9


def test_unchanged_state_does_not_emit_for_each_sensor_sample():
    emitted=[]
    logger=SimpleNamespace(info=lambda _:None)
    stub=SimpleNamespace(state='WAITING',reason='待機',p={'service_timeout_sec':5.},
                         get_logger=lambda:logger,now=lambda:10.,
                         publish_status=lambda:emitted.append(True))
    HeadingInitializerNode.set_state(stub,'WAITING','待機')
    assert not emitted
    HeadingInitializerNode.set_state(stub,'COLLECTING','平均中')
    assert len(emitted)==1
    HeadingInitializerNode.set_state(stub,'COLLECTING','平均中')
    assert len(emitted)==1


def test_datum_service_timeout_starts_when_fix_arrives():
    clock = [100.]
    errors = []
    stub = SimpleNamespace(
        state='DATUM_SERVICE', now=lambda: clock[0],
        stationary=lambda: (True, ''), fix=None,
        datum_service_wait_since=None, deadline=1.,
        p={'service_timeout_sec': 5.},
        datum_service=SimpleNamespace(service_is_ready=lambda: False),
        fail=errors.append)
    HeadingInitializerNode.tick(stub)
    stub.fix = (object(), clock[0])
    HeadingInitializerNode.tick(stub)
    assert not errors
    clock[0] += 5.1
    stub.fix = (object(), clock[0])
    HeadingInitializerNode.tick(stub)
    assert errors == ['datum serviceが利用できません']


def test_datum_response_uses_configured_gps_verification_timeout():
    states = []
    stub = SimpleNamespace(
        pending=SimpleNamespace(done=lambda: True, result=lambda: None),
        now=lambda: 10., p={'service_timeout_sec': 5., 'gps_verify_timeout_sec': 30.},
        set_state=lambda state, reason: states.append((state, reason)))
    HeadingInitializerNode.poll_call(stub, 'GPS_VERIFY', '確認')
    assert stub.verify_after == 10.
    assert stub.deadline == 40.
    assert states == [('GPS_VERIFY', '確認')]


def test_gps_verify_timeout_reports_missing_input():
    errors = []
    stub = SimpleNamespace(
        state='GPS_VERIFY', now=lambda: 41., stationary=lambda: (True, ''),
        gps=None, verify_after=10., deadline=40.,
        fromll_service=SimpleNamespace(service_is_ready=lambda: True),
        fail=errors.append)
    HeadingInitializerNode.tick(stub)
    assert errors == ['確認できません: datum後のGNSS変換出力']
