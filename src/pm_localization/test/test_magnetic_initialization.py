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
