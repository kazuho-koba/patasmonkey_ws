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
