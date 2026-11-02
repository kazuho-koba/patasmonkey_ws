"""曲率参照速度の減速・再加速と弧長参照の挙動を確認する。"""
import numpy as np
from pm_planner.mppi_poc import MppiConfig,MppiPoc
from pm_planner.reference import Limits,build_reference


def test_corner_brakes_before_curve_and_recovers_on_straight():
    points=[(0.,0.),(4.,0.),(4.,4.)]
    wp=[dict(waypoint_id=str(i),attribute='start' if i==0 else 'goal' if i==2 else 'pass',pause_sec=0.,note='') for i in range(3)]
    ref=build_reference(points,wp,Limits(.5,1.,1.,1.))
    solver=MppiPoc(MppiConfig(target_speed=1.,min_radius=.4,use_reference_speed_profile=True))
    solver.set_path([(x,y) for x,y,_,_ in ref.samples])
    speed=solver.reference_speed;station=solver.s
    assert np.max(speed)<=1.
    assert np.isfinite(speed).all() and np.min(speed)>=0.
    assert speed[-1]==0.
    # 角の手前で減速し、角を抜けて十分離れた直線では目標へ復帰する。
    assert np.interp(3.5,station,speed)<np.interp(2.,station,speed)
    assert np.interp(6.,station,speed)>.8
    assert np.max(speed*np.abs(solver.reference_curvature))<=1.+1e-9
    accel=.4
    assert np.all(np.abs(np.diff(speed**2))<=2.*accel*np.diff(station)+1e-8)


def test_profile_rollout_keeps_initial_command_bounded():
    cfg=MppiConfig(target_speed=1.,use_reference_speed_profile=True)
    solver=MppiPoc(cfg);solver.set_path([(0.,0.),(5.,0.)])
    r=solver.step((0.,0.,0.),(0.,0.),previous_command=(0.,0.))
    assert np.isfinite(r['trajectory']).all()
    assert 0.<=r['command'][0]<=cfg.max_yaw_accel*cfg.min_radius*cfg.dt+1e-9
