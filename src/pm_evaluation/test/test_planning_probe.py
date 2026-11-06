"""診断用solver復元が乱数・warm start・地図の条件を保存していることを確認する。"""
from dataclasses import asdict
import json
import numpy as np
from pm_evaluation.planning_probe import restore_solver,environment
from pm_planner.mppi_poc import MppiPoc,MppiConfig


def snapshot(solver):
    return dict(config=np.asarray(json.dumps(asdict(solver.c))),reference=solver.path.copy(),
        nominal=solver.nominal.copy(),station=np.asarray(solver.station),
        rng=np.asarray(json.dumps(solver.rng.bit_generator.state)),required_station=np.asarray('null'),
        obstacle=np.zeros((160,160),dtype=np.int8),terrain=np.zeros((160,160),dtype=np.int8),
        mode=np.asarray('obstacle_only'),resolution=np.asarray(.1),origin=np.array([-8.,-8.,0.]),radius=np.asarray(.36))


def test_restores_nonzero_nominal_and_advanced_random_state():
    solver=MppiPoc(MppiConfig(target_speed=.3,steps=35,dt=.1,tracking_mode='corridor',path_tolerance_m=.6,min_radius=.4))
    solver.set_path([(0.,0.),(8.,0.)]);data=snapshot(solver)
    solver.step((0.,0.,0.),(.3,0.),environment=environment(data))
    data=snapshot(solver)
    expected=solver.step((.1,0.,0.),(.3,0.),previous_command=(.3,0.),environment=environment(data))
    actual=restore_solver(data).step((.1,0.,0.),(.3,0.),previous_command=(.3,0.),environment=environment(data))
    assert np.array_equal(expected['trajectory'],actual['trajectory'])


def test_far_obstacle_does_not_change_short_prediction():
    solver=MppiPoc(MppiConfig(target_speed=.3,steps=35,dt=.1,tracking_mode='corridor',path_tolerance_m=.6,min_radius=.4))
    solver.set_path([(0.,0.),(8.,0.)]);data=snapshot(solver)
    free=restore_solver(data).step((0.,0.,0.),(.3,0.),environment=environment(data))
    data['obstacle'][75:86,104:107]=100  # 約2.5m先。短いhorizonの軌道にはかからない
    blocked=restore_solver(data).step((0.,0.,0.),(.3,0.),environment=environment(data))
    assert np.array_equal(free['trajectory'],blocked['trajectory'])


def test_snapshot_replays_explicit_margin_and_legacy_snapshot_remains_compatible():
    solver=MppiPoc(MppiConfig());solver.set_path([(0.,0.),(8.,0.)])
    data=snapshot(solver);data['obstacle'][80,80]=100
    old=environment(data)
    data['footprint_margin_m']=np.asarray(.25)
    new=environment(data)
    assert old.margin_m==float(data['resolution'])
    assert new.margin_m==.25
    assert old.sample(.55,.05)==0 and new.sample(.55,.05)==100
