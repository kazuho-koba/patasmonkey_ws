"""保持方式・配列の大小でunknown率の分母を変えないことを確認する。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


@pytest.fixture
def evaluation(monkeypatch):
    tools=Path(__file__).resolve().parents[1]/'tools'
    monkeypatch.syspath_prepend(str(tools))
    spec=importlib.util.spec_from_file_location('stride_coverage',tools/'sweep_pixel_stride_coverage.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_map_outside_is_unknown_in_fixed_distance_band(evaluation):
    """従来の8m四方に入らない5m帯も、分母から消さずunknownに数える。"""
    model=SimpleNamespace(res=.1,origin=np.array([-4.,-4.]),
        costs=lambda:(np.zeros((80,80),dtype=np.int8),None))
    result=evaluation.coverage(model,(0,0.,0.,0.))
    assert result['d5_cells']>0 and result['d5_unknown']==result['d5_cells']
    assert result['d1_free']==result['d1_cells']


@pytest.mark.parametrize('yaw',[0.,.4,1.2])
def test_denominator_is_independent_of_grid_extent(evaluation,yaw):
    """異なる外接矩形でも同じ車体基準距離帯のodomセル数を比較する。"""
    def model(size,origin):
        return SimpleNamespace(res=.1,origin=np.array(origin),
            costs=lambda:(np.full((size,size),-1,dtype=np.int8),None))
    a=evaluation.coverage(model(80,[-4.,-4.]),(0,-.03,.07,yaw))
    b=evaluation.coverage(model(120,[-5.,-5.]),(0,-.03,.07,yaw))
    for d in evaluation.BANDS:
        assert a[f'd{d}_cells']==b[f'd{d}_cells']
        assert a[f'd{d}_unknown']==a[f'd{d}_cells']
        assert b[f'd{d}_unknown']==b[f'd{d}_cells']
