"""非駆動MPPIの制約・不正入力・閉ループの基礎試験。"""
import math
import csv
import numpy as np
import pytest

from pm_planner.mppi_poc import MppiConfig, MppiPoc
from pm_planner.mppi_sim import simulate


def test_deterministic_and_hard_constraints():
    c=MppiConfig(batch_size=128,steps=20)
    a=MppiPoc(c);b=MppiPoc(c)
    for solver in (a,b):solver.set_path([(0.,0.),(2.,0.),(3.,1.)])
    v=w=x=y=yaw=0.
    for _ in range(20):
        r=a.step((x,y,yaw),(v,w));other=b.step((x,y,yaw),(v,w))
        assert r['command']==pytest.approx(other['command'])
        nv,nw=r['command'];assert 0<=nv<=c.max_speed
        assert abs(nw)<=min(c.max_yaw_rate,nv/c.min_radius)+1e-12
        assert abs(nv-v)<=c.max_yaw_accel*c.min_radius*c.dt+1e-12
        assert abs(nw-w)<=c.max_yaw_accel*c.dt+1e-12
        yaw+=nw*c.dt;x+=nv*math.cos(yaw)*c.dt;y+=nv*math.sin(yaw)*c.dt;v,w=nv,nw


def test_invalid_path_pose_and_projection_limits():
    a=MppiPoc()
    with pytest.raises(ValueError):a.step((0.,0.,0.),(0.,0.))
    for path in ([(0.,0.),(0.,0.)],[(0.,0.),(float('nan'),0.)]):
        with pytest.raises(ValueError):a.set_path(path)
    a.set_path([(0.,0.),(5.,0.),(5.,.3),(0.,.3)])
    with pytest.raises(ValueError):a.step((0.,3.,0.),(0.,0.))
    with pytest.raises(ValueError):a.step((float('nan'),0.,0.),(0.,0.))
    with pytest.raises(ValueError):a.step((0.,0.,0.),(-.5,0.))
    result=a.step((0.,.3,0.),(0.,0.))
    assert result['station']<.1


@pytest.mark.parametrize('name,points',[
    ('straight',[(0.,0.),(5.,0.)]),('corner',[(0.,0.),(3.,0.),(3.,3.)]),
    ('turn',[(0.,0.),(3.,0.),(.1,.05)]),
    ('parallel_return',[(0.,0.),(3.,0.),(.01,.2),(-2.,.2)])])
def test_closed_loop_reaches_end_without_path_jump(tmp_path,name,points):
    result=simulate(name,points,tmp_path,MppiConfig())
    assert result['goal'],result
    assert result['progress_state']=='COMPLETE',result
    assert result['waypoints_completed']==result['waypoints_total']
    assert result['station_m']>=result['path_length_m']-.25
    # 非駆動PoCの回帰限度。自己交差時に対応区間の誤差を過小評価する点に注意。
    assert result['max_nearest_error_m']<.2
    with (tmp_path/(name+'.csv')).open() as stream:
        rows=list(csv.DictReader(stream))
    # CSVの時刻はpose観測と同じstep開始時刻で、指令適用後のposeは次行に載る。
    first,second=rows[:2]
    assert float(first['time_s'])==0.
    assert float(second['time_s'])==pytest.approx(.1)
    next_yaw=float(first['yaw_rad'])+float(first['w_radps'])*.1
    assert float(second['x_m'])==pytest.approx(float(first['x_m'])+float(first['v_mps'])*math.cos(next_yaw)*.1)
    assert float(second['y_m'])==pytest.approx(float(first['y_m'])+float(first['v_mps'])*math.sin(next_yaw)*.1)
