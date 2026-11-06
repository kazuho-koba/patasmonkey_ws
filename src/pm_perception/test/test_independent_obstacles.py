"""初回保護、欠測中立、N平均・欠測窓の解除、channel分離を確認する。"""
import numpy as np
import pytest
from pm_perception.independent_obstacles import IndependentObstacles


def floor_points():
    return np.array([[x,y,0.] for x in np.arange(-.55,.66,.1)
                     for y in np.arange(-.55,.66,.1) for _ in range(3)])


def test_first_hit_and_missing():
    m=IndependentObstacles(size=2.)
    p=np.vstack((floor_points(),[[.05,.05,.2]]))
    m.update(p,[-1.,.05,.2],[0,0],1)
    assert m.cells[(0,0)]['obstacle']==100
    m.update(np.empty((0,3)),[-1.,.05,.2],[0,0],2)
    assert m.cells[(0,0)]['obstacle']==100
    m.update(floor_points(),[-1.,.05,.2],[0,0],3)
    assert m.cells[(0,0)]['obstacle']==100


def test_low_window_clear_and_reappearance():
    m=IndependentObstacles(size=2.)
    base=floor_points();hit=np.array([[.05,.05,.2]])
    m.update(np.vstack((base,hit)),[-1.,.05,.2],[0,0],1)
    # 背景rayは要求しない。同じセルの低い再観測だけで窓を更新する。
    for stamp in (2,):
        m.update(base,[-1.,.05,.2],[0,0],stamp)
        assert m.cells[(0,0)]['active']
    m.update(base,[-1.,.05,.2],[0,0],3)
    assert not m.cells[(0,0)]['active']
    m.update(np.vstack((base,hit)),[-1.,.05,.2],[0,0],5)
    assert m.cells[(0,0)]['active']


def test_rolling_bound_and_channels():
    m=IndependentObstacles(size=2.)
    m.update(floor_points(),[0,0,.4],[0,0],1)
    a,b=m.costs();assert a is not b and a.shape==(20,20)
    m.update(floor_points(),[10,0,.4],[10,0],2)
    assert len(m.cells)==0
    with pytest.raises(ValueError):m.update(floor_points(),[0,0,.4],[0,0],2)


def test_missing_window_clears_to_unknown():
    m=IndependentObstacles(size=2.)
    m.update(np.vstack((floor_points(),[[.05,.05,.2]])),[-1.,.05,.2],[0,0],1)
    for stamp in (2,3,4):
        m.update(np.empty((0,3)),[-1.,.05,.2],[0,0],stamp)
    assert not m.cells[(0,0)]['active']
    assert m.cells[(0,0)]['obstacle']==-1
