"""TFの3次元回転と時刻対応の基本的な挙動を検証する。"""
import math
import numpy as np
import pytest

from pm_evaluation.cli.analyze_localization_tf import nearest_index, rotate


def test_rotate_full_3d():
    q=(0.,0.,math.sin(math.pi/4),math.cos(math.pi/4))
    assert rotate(q,(1.,0.,2.)).tolist() == pytest.approx([0.,1.,2.])
    with pytest.raises(ValueError):
        rotate((0,0,0,0),(1,0,0))


def test_nearest_time():
    times=np.array([1.,2.,3.])
    assert nearest_index(times,0.)==0
    assert nearest_index(times,2.7)==2
    assert nearest_index(times,1.2)==0


def test_transform_preserves_local_gps_difference():
    # 同じTFをlocalとGPSにかけると、globalとの相対距離は元の差と一致する。
    q=(0.,0.,math.sin(.4),math.cos(.4))
    translation=np.array([1000.,-2000.,100.])
    local=np.array([7.,5.,2.])
    gps=np.array([8.,4.,3.])
    global_position=rotate(q,local)+translation
    gps_map=rotate(q,gps)+translation
    assert np.linalg.norm(gps_map-global_position) == pytest.approx(np.linalg.norm(gps-local))
