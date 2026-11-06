"""footprint・unknown・線分検査とMPPIの環境cost接続を確認する。"""
import numpy as np
import pytest
from pm_planner.terrain_cost import TerrainCost
from pm_planner.mppi_poc import MppiConfig,MppiPoc


def grid(obstacle=False):
    a=np.zeros((120,120),dtype=int)
    if obstacle:a[60,70]=100
    return TerrainCost(a.ravel(),120,120,.05,(-3.,-3.,0.),.2)


def test_footprint_and_unknown_and_rotated_origin():
    env=grid(True)
    assert env.sample(np.array([.5]),np.array([0.]))[0]==100
    assert env.sample(np.array([.35]),np.array([0.]))[0]==100
    assert env.sample(np.array([0.]),np.array([0.]))[0]==0
    assert env.segment(np.array([0.]),np.array([0.]),np.array([1.]),np.array([0.]))[0]==100
    assert env.sample(5.,0.)==100
    a=np.zeros((10,10),int);a[5,5]=-1
    rotated=TerrainCost(a.ravel(),10,10,.1,(1.,2.,np.pi/2),.01)
    assert rotated.sample(.45,2.55)==100


def test_environment_blocks_unknown_or_obstacle_and_allows_clear_map():
    c=MppiConfig(batch_size=64,steps=10,target_speed=.5,tracking_mode='corridor')
    solver=MppiPoc(c);solver.set_path([(0.,0.),(2.,0.)])
    result=solver.step((0.,0.,0.),(0.,0.),environment=grid())
    assert result['command'][0]>0
    blocked=TerrainCost(np.full(120*120,-1),120,120,.05,(-3.,-3.,0.),.2)
    with pytest.raises(ValueError,match='候補軌道'):solver.step((0.,0.,0.),(0.,0.),environment=blocked)


def test_corridor_offset_is_not_forced_back_to_centreline():
    c=MppiConfig(batch_size=384,steps=25,target_speed=.5,tracking_mode='corridor',max_path_error=2.)
    solver=MppiPoc(c);solver.set_path([(0.,0.),(20.,0.)])
    command=solver.step((0.,.8,0.),(0.,0.))['command']
    assert abs(command[1])<.06


@pytest.mark.parametrize('resolution',[.05,.1])
@pytest.mark.parametrize('hazard',[100,-1])
def test_explicit_metric_margin_is_independent_of_resolution(resolution,hazard):
    """同じm設定なら解像度を変えても物理的な膨張範囲を維持する。"""
    width=int(round(4./resolution));a=np.zeros((width,width),int);a[width//2,width//2]=hazard
    env=TerrainCost(a.ravel(),width,width,resolution,(-2.,-2.,0.),.3,margin_m=.1)
    assert env.inflation_radius==pytest.approx(.4)
    assert env.sample(.35,.025)==100 and env.sample(.55,.025)==0


def test_zero_and_larger_margin_change_clearance():
    a=np.zeros((40,40),int);a[20,20]=100
    narrow=TerrainCost(a.ravel(),40,40,.1,(-2.,-2.,0.),.1,margin_m=0.)
    wide=TerrainCost(a.ravel(),40,40,.1,(-2.,-2.,0.),.1,margin_m=.2)
    assert narrow.sample(.25,.05)==0 and wide.sample(.25,.05)==100


@pytest.mark.parametrize('margin',[-.01,float('nan'),float('inf')])
def test_invalid_margin_is_rejected(margin):
    with pytest.raises(ValueError):TerrainCost([0]*100,10,10,.1,(0.,0.,0.),.1,margin_m=margin)



def test_zero_margin_keeps_decimal_radius_boundary():
    a=np.zeros((40,40),int);a[20,20]=100
    env=TerrainCost(a.ravel(),40,40,.1,(-2.,-2.,0.),.3,margin_m=0.)
    assert env.sample(.35,.05)==100  # セル中心間0.3 mを丸めで除外しない
    assert env.sample(.45,.05)==0


def test_clearance_gradient_and_unknown_and_margin():
    """障害物とunknownを同じ距離尺度で扱い、希望離隔外は無罰にする。"""
    a=np.zeros((100,100),int);a[:,50]=100
    env=TerrainCost(a.ravel(),100,100,.05,(-2.5,-2.5,0.),.3,margin_m=.1)
    risks=env.clearance_risk(np.array([-.45,-.7,-1.2]),np.zeros(3),.6)
    assert risks[0]>risks[1]>risks[2] and risks[2]==0
    a[:,50]=-1
    unknown=TerrainCost(a.ravel(),100,100,.05,(-2.5,-2.5,0.),.3,margin_m=.1)
    assert np.allclose(risks,unknown.clearance_risk(np.array([-.45,-.7,-1.2]),np.zeros(3),.6))
    assert env.clearance_risk(20.,0.,.6)==1
    assert env.clearance_risk(-.45,0.,0.)==0


def test_clearance_speed_cap_and_deceleration():
    """壁と並行する既知free経路で低速化し、既に高速なら減速度を守る。"""
    a=np.zeros((160,160),int);a[80,:]=100
    env=TerrainCost(a.ravel(),160,160,.05,(-4.,-4.,0.),.3,margin_m=.1)
    cfg=MppiConfig(batch_size=64,steps=15,target_speed=1.,tracking_mode='corridor',
                   clearance_distance_m=.6,clearance_cost_weight=3.,clearance_min_speed_mps=.2)
    solver=MppiPoc(cfg);solver.set_path([(0.,-.45),(3.,-.45)])
    result=solver.step((0.,-.45,0.),(.2,0.),environment=env)
    risk=float(env.clearance_risk(0.,-.45,.6))
    assert result['command'][0]<=1.-.8*risk+1e-9
    solver=MppiPoc(cfg);solver.set_path([(0.,-.45),(3.,-.45)])
    fast=solver.step((0.,-.45,0.),(.8,0.),environment=env)
    assert fast['command'][0]>=.8-cfg.max_yaw_accel*cfg.min_radius*cfg.dt-1e-9


@pytest.mark.parametrize('name',['clearance_distance_m','clearance_cost_weight','clearance_min_speed_mps'])
def test_clearance_parameter_validation(name):
    for value in (-1.,float('nan'),float('inf')):
        with pytest.raises(ValueError):MppiConfig(**{name:value})


def test_clearance_weight_prefers_wider_route():
    """同一seed・減速条件でも離隔costを足すと平行壁から離れる候補になる。"""
    from dataclasses import replace
    a=np.zeros((120,120),int);a[70,:]=100
    env=TerrainCost(a.ravel(),120,120,.05,(-3.,-3.,0.),.3,margin_m=.1)
    c=MppiConfig(target_speed=1.,tracking_mode='corridor')
    lateral=[]
    for weight in (0.,3.):
        solver=MppiPoc(replace(c,clearance_cost_weight=weight))
        solver.set_path([(0.,0.),(3.,0.)])
        result=solver.step((0.,0.,0.),(.2,0.),environment=env)
        lateral.append(result['trajectory'][-1,1])
    assert lateral[1]<lateral[0]-.1
