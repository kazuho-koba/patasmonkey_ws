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


def high_points(keys):
    """局所平面に依存せず、各指定セルの高さ幅で危険を作る。"""
    return np.array([[x+.05,y+.05,z] for x,y in keys for z in (0.,.2)])


def test_body_window_front_extent_and_rotation_keeps_world_cell():
    """同じ絶対セルの履歴は回転しない。範囲外だけ破棄し、前方6mを確保する。"""
    m=IndependentObstacles(size=8.,retention_mode='body_aligned',forward_offset_m=2.)
    points=high_points([(3.,0.),(5.5,0.),(-2.5,0.),(0.,5.5)])
    m.update(points,[0,0,.4],[0,0],1)
    assert (30,0) in m.cells and (55,0) in m.cells
    assert (-25,0) not in m.cells and (0,55) not in m.cells
    previous=m.cells[(30,0)]
    m.update(points,[0,0,.4],[0,0],2,np.pi/2)
    assert m.cells[(30,0)] is previous  # 窓履歴も同じセルに残る
    assert (55,0) not in m.cells and (0,55) in m.cells
    assert (-25,0) in m.cells
    assert m.costs()[0].shape==(m.height,m.width)


def test_diagonal_window_includes_front_and_masks_bounding_corners():
    """45度旋回でも前方端を切らず、外接矩形の角は未知のままにする。"""
    m=IndependentObstacles(size=8.,retention_mode='body_aligned',forward_offset_m=2.)
    m.update(high_points([(4.,4.),(-3.,-3.)]),[0,0,.4],[0,0],1,np.pi/4)
    assert (40,40) in m.cells and (-30,-30) not in m.cells
    assert m.width>80 and m.height>80
    origin,mask=m.retention_window([0,0],np.pi/4)
    obstacle,_=m.costs()
    assert (obstacle[~mask]==-1).all()
    # 斜め窓の実セル数は約80²。外接矩形全体を既知として埋めない。
    assert 6200<mask.sum()<6600
    legacy=IndependentObstacles(size=8.,retention_mode='odom_aligned',forward_offset_m=2.)
    legacy.update(high_points([(4.,4.)]),[0,0,.4],[0,0],1,np.pi/4)
    assert not legacy.cells and legacy.costs()[0].shape==(80,80)


def test_retention_bounds_follow_translation_and_legacy_ignores_offset():
    """負座標・非セル境界のposeでも絶対XYセルで追従する。旧方式のoffsetは無効。"""
    a=IndependentObstacles(size=8.)
    b=IndependentObstacles(size=8.,forward_offset_m=2.)
    for stamp,center in enumerate(([-.03,.07],[1.03,-2.07]),1):
        a.update(floor_points(),[0,0,.4],center,stamp,.7)
        b.update(floor_points(),[0,0,.4],center,stamp,.7)
        assert np.array_equal(a.origin,b.origin)
        assert np.array_equal(a.costs(),b.costs())
    with pytest.raises(ValueError):IndependentObstacles(retention_mode='invalid')
    with pytest.raises(ValueError):IndependentObstacles(forward_offset_m=5.)


@pytest.mark.parametrize('yaw',[0.,np.pi/4,np.pi/2,np.pi,-np.pi/3])
def test_body_extents_at_arbitrary_heading_and_negative_pose(yaw):
    """前後・左右の距離制限はodom軸との角度や車体位置で変わらない。"""
    m=IndependentObstacles(size=8.,retention_mode='body_aligned',forward_offset_m=2.)
    center=np.array([-1.03,2.07]);c,s=np.cos(yaw),np.sin(yaw)
    local=np.array([[5.5,0],[-1.5,0],[0,3.5],[0,-3.5],
                    [6.5,0],[-2.5,0],[0,4.5],[0,-4.5]])
    world=local@np.array([[c,s],[-s,c]])+center
    keys=np.floor(world/.1).astype(int)
    points=np.array([[(x+.5)*.1,(y+.5)*.1,z] for x,y in keys for z in (0.,.2)])
    m.update(points,[0,0,.4],center,1,yaw)
    assert all(tuple(key) in m.cells for key in keys[:4])
    assert all(tuple(key) not in m.cells for key in keys[4:])
