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
