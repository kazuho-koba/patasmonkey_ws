#!/usr/bin/env python3
"""実機非接続で制約を照合し、MPPIの遅延・モデル誤差に対する追従を測る。

vehicle interfaceから数値計算メソッドだけをASTで抽出する。nodeやモータドライバは
importも初期化もせず、motor出力はメモリ内でのみ扱う。実機走行の代替ではない。
"""
import argparse
import ast
from collections import deque
from dataclasses import asdict
import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from pm_planner.io import read_limits, read_vehicle_parameters
from pm_planner.mppi_poc import MppiConfig, MppiPoc
from pm_planner.reference import build_reference
from pm_planner.progress import Progress
from pm_planner.mppi_sim import save_svg


class Stamp:
    """制御クロックをナノ秒で再現する。"""
    def __init__(self, value):self.nanoseconds=int(value)
    def __sub__(self, other):return Stamp(self.nanoseconds-other.nanoseconds)


class Twist:
    def __init__(self):
        self.linear=SimpleNamespace(x=0.)
        self.angular=SimpleNamespace(z=0.)


def make_interface(source, parameters):
    """製品の計算コードを抽出し、USBアクセスに到達しないオブジェクトを作る。"""
    tree=ast.parse(Path(source).read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='VehicleInterfaceNode')
    names={'clamp','reset_motion_limits','apply_motion_limits','motor_control'}
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names]
    if len(methods)!=len(names):raise ValueError('vehicle interfaceの構成変更を確認してください')
    tree=ast.Module(body=[ast.ClassDef(name='NumericalControl',bases=[],keywords=[],body=methods,decorator_list=[])],type_ignores=[])
    env={'math':math,'Twist':Twist}
    exec(compile(ast.fix_missing_locations(tree),str(source),'exec'),env)
    obj=env['NumericalControl']()
    for k,v in parameters.items():setattr(obj,k,v)
    obj.now=0.
    obj.get_clock=lambda:SimpleNamespace(now=lambda:Stamp(round(obj.now*1e9)))
    obj.odrive_connected=False
    obj.reset_motion_limits()
    return obj


def apply(control, command, dt):
    control.now+=dt
    msg=Twist();msg.linear.x,msg.angular.z=command
    control.motor_control(msg,min(control.max_linear_speed,control.autonomous_max_linear_speed))
    return np.array([control._selected_cmd_vel.linear.x,control._selected_cmd_vel.angular.z])


def audit(source,p,limits):
    """左右旋回・加減速・停止・車輪上限を製品メソッドへ入力して確認する。"""
    control=make_interface(source,p);samples=[];previous=np.zeros(2)
    for target in [(1.,0.),(1.,2.),(1.,-2.),(.1,1.),(3.,3.),(0.,0.)]:
        for _ in range(30):
            v,w=apply(control,target,.1)
            assert abs(v)<=limits.max_speed+1e-9
            assert abs(w)<=limits.max_yaw_rate+1e-9
            assert abs(w)<=abs(v)/limits.radius+1e-9
            wheel_yaw=w/p['skid_steer_yaw_efficiency']
            wheels=(np.array([v-wheel_yaw*p['tread_width']/2,v+wheel_yaw*p['tread_width']/2])/(2*math.pi*p['wheel_radius']))
            assert max(abs(wheels))<=p['max_whl_rps']+1e-8
            samples.append([*target,v,w,*wheels])
            previous[:]=(v,w)
    return dict(n=len(samples),constraints_passed=True,effective_limits=asdict(limits),
                max_observed_yaw_rate=float(np.max(np.abs(np.array(samples)[:,3]))),
                straight_linear_accel=p['max_yaw_accel']*p['min_turn_radius'],
                planner_linear_accel=limits.max_yaw_accel*limits.radius,
                notes=['停止・半径の絶対制約はyaw加速度より優先',
                       '車体の半径・yaw上限はYAMLどおり。旋回効率による縮小は行わない',
                       '曲率に応じた滑らかな減速と車輪上限は実interface側で評価。MPPIの近似モデルとは一致しない'])


def stop_probe(source,p,control,velocity,scenario):
    """走行中の状態を複製し、ゼロ要求からの遅延・応答による移動距離を測る。

    本試行は継続する。GUI切断等の検出時間や実行管理はこの試験に含まない。
    """
    clone=make_interface(source,p)
    clone.now=control.now
    clone.prev_linear_speed_cmd=control.prev_linear_speed_cmd
    clone.prev_yaw_rate_cmd=control.prev_yaw_rate_cmd
    clone.prev_yaw_rate_time=control.prev_yaw_rate_time
    v=velocity.copy();previous=np.array([clone.prev_linear_speed_cmd,clone.prev_yaw_rate_cmd])
    elapsed=distance=0.;tau=scenario.get('tau',0.)
    for _ in range(150):
        requested=previous if elapsed<scenario.get('command_delay',0.) else np.zeros(2)
        desired=apply(clone,requested,.02)
        v+=(desired-v)*(1.-math.exp(-.02/tau) if tau else 1.)
        distance+=abs(v[0])*.02;elapsed+=.02
        if abs(v[0])<.03 and elapsed>=scenario.get('command_delay',0.):break
    return dict(distance_m=float(distance),time_s=elapsed,initial_speed_mps=float(velocity[0]))


def simulate(name,points,scenario,root,source,p,limits,seed=42,target_speed=.15,use_reference_speed_profile=False,max_steps=1500,waypoints=None):
    """固定MPPI条件と異なる車体条件を閉ループにし、真のposeで誤差を測る。"""
    wp=waypoints if waypoints is not None else [dict(waypoint_id=str(i),attribute='start' if i==0 else 'goal' if i==len(points)-1 else 'pass',pause_sec=0.,note='') for i in range(len(points))]
    ref=build_reference(points,wp,limits.with_turn_margin(.1))
    c=MppiConfig(min_radius=limits.radius,max_speed=limits.max_speed,max_yaw_rate=limits.max_yaw_rate,max_yaw_accel=limits.max_yaw_accel,seed=seed,target_speed=target_speed,use_reference_speed_profile=use_reference_speed_profile)
    solver=MppiPoc(c);solver.set_path([(q[0],q[1]) for q in ref.samples]);control=make_interface(source,p)
    rng=np.random.default_rng(seed)
    x,y,yaw=ref.samples[0][:3];y+=scenario.get('start_offset',0.);yaw+=scenario.get('start_yaw',0.)
    velocity=np.zeros(2);previous=np.zeros(2);queue=deque();history=deque();time=0.;rows=[];failure='';goal=False
    progress=Progress(ref);progress.start((0.,x,y,yaw,0.));saturation=[];braking=None
    for step in range(max_steps):
        dt=([.05,.15,.1][step%3] if scenario.get('jitter') else .1)
        history.append((time,x,y,yaw,*velocity))
        while len(history)>1 and history[1][0]<=time-scenario.get('pose_delay',0.):history.popleft()
        obs=history[0] if scenario.get('pose_delay') else history[-1]
        pose=np.array(obs[1:4])+rng.normal(0.,[scenario.get('noise_xy',0.)]*2+[scenario.get('noise_yaw',0.)])
        # 終点検出後は再追従へ戻さずゼロ要求を継続し、遅延した車体の停止も待つ。
        # 実行管理は未接続のため、これは評価器内だけの終点停止手順である。
        try:r=(dict(command=(0.,0.),goal=True) if goal else
               solver.step(pose,obs[4:6],previous_command=tuple(previous)))
        except ValueError as exc:failure=str(exc);break
        request=np.array(r['command']);queue.append((time,request))
        delay=scenario.get('command_delay',0.)
        while len(queue)>1 and queue[1][0]<=time-delay:queue.popleft()
        delivered=queue[0][1] if queue[0][0]<=time-delay else np.zeros(2)
        previous=apply(control,delivered,dt)
        desired=previous.copy();desired[1]*=scenario.get('yaw_gain',1.)
        tau=scenario.get('tau',0.)
        velocity+=(desired-velocity)*(1.-math.exp(-dt/tau) if tau else 1.)
        if braking is None and time>=5. and abs(velocity[0])>.1:
            braking=stop_probe(source,p,control,velocity,scenario)
        saturation.append(float(np.linalg.norm(request-previous)))
        # 誤差は直近stationに対応する近傍の線分で計算し、近接復路への飛びを抑える。
        j=int(np.searchsorted(solver.s,solver.station));lo=max(0,j-8);hi=min(len(solver.path)-1,j+8)
        a=solver.path[lo:hi];b=solver.path[lo+1:hi+1]-a
        fraction=np.clip(np.sum((np.array([x,y])-a)*b,axis=1)/np.sum(b*b,axis=1),0.,1.)
        error=float(np.min(np.linalg.norm(a+fraction[:,None]*b-np.array([x,y]),axis=1)))
        # 要求・interface採用後・人工車体の速度を区別して保存する。
        rows.append((time,x,y,yaw,*request,solver.station,error,*previous,*velocity))
        yaw+=velocity[1]*dt;x+=velocity[0]*math.cos(yaw)*dt;y+=velocity[0]*math.sin(yaw)*dt;time+=dt
        progress.update((time,x,y,yaw,velocity[0]))
        if progress.state=='FAULT':failure='progress:'+progress.reason;break
        if r['goal']:goal=True
        if goal and progress.state=='COMPLETE':break
    # 通常追従と別にGUI停止等からゼロ要求が届いた後の距離を測る。通信断検出時間は含まない。
    stop_start=np.array([x,y]);stop_distance=0.;stop_time=0.;prior=stop_start.copy()
    tau=scenario.get('tau',0.)
    for _ in range(100):
        accepted=apply(control,(0.,0.),.02)
        velocity+=(accepted-velocity)*(1.-math.exp(-.02/tau) if tau else 1.)
        yaw+=velocity[1]*.02;x+=velocity[0]*math.cos(yaw)*.02;y+=velocity[0]*math.sin(yaw)*.02
        stop_distance+=float(np.linalg.norm(np.array([x,y])-prior));prior=np.array([x,y]);stop_time+=.02
        if abs(velocity[0])<.03:break
    key=name+'_'+scenario['name']
    with (root/(key+'.csv')).open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['time_s','x_m','y_m','yaw_rad','v_request','w_request','station_m','error_m','v_accepted','w_accepted','v_actual','w_actual']);writer.writerows(rows)
    save_svg(root/(key+'.svg'),solver.path,rows)
    errors=[q[7] for q in rows]
    result=dict(route=name,scenario=scenario,seed=seed,target_speed_mps=target_speed,use_reference_speed_profile=use_reference_speed_profile,goal=goal,progress=progress.state,waypoints=len(progress.completed),
                total_waypoints=len(ref.events),failure=failure or ('' if goal and progress.state=='COMPLETE' else 'simulation_timeout'),duration_s=time,
                max_error_m=max(errors) if errors else None,p95_error_m=float(np.percentile(errors,95)) if errors else None,
                mean_interface_change=float(np.mean(saturation)) if saturation else None,
                max_accepted_speed_mps=max((abs(q[8]) for q in rows),default=0.),
                max_accepted_yaw_radps=max((abs(q[9]) for q in rows),default=0.),
                stop_distance_m=stop_distance,stop_time_s=stop_time,moving_stop_probe=braking)
    print(key,goal,result['failure'],result['max_error_m'],flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,default=Path('/workspaces/patasmonkey_ws'))
    parser.add_argument('--vehicle-config',type=Path,help='単独試験用の車両設定。未指定は共通pm_configを使用')
    parser.add_argument('--target-speed',type=float,default=.15,help='MPPI目標速度[m/s]。旋回時はinterfaceの制限を適用する')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--reference-speed-profile',dest='reference_speed_profile',action='store_true',help='曲率参照速度を使う（既定）')
    mode.add_argument('--constant-speed-reference',dest='reference_speed_profile',action='store_false',help='従来の一定参照速度を対照に使う')
    parser.set_defaults(reference_speed_profile=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output;root.mkdir(parents=True,exist_ok=False)
    config=args.vehicle_config or args.workspace/'src/pm_config/config/vehicle_control.yaml'
    source=args.workspace/'src/pm_control/pm_control/vehicle_interface_node.py'
    p=read_vehicle_parameters(config);limits=read_limits(config)
    check=audit(source,p,limits)
    (root/'audit.json').write_text(json.dumps(check,ensure_ascii=False,indent=2))
    scenarios=[dict(name='baseline'),dict(name='command_delay_100ms',command_delay=.1),
        dict(name='command_delay_300ms',command_delay=.3),dict(name='yaw_gain_080',yaw_gain=.8),
        dict(name='yaw_gain_120',yaw_gain=1.2),dict(name='pose_noise',noise_xy=.03,noise_yaw=math.radians(2)),
        dict(name='jitter',jitter=True),dict(name='start_error',start_offset=.1,start_yaw=math.radians(10)),
        dict(name='combined',command_delay=.2,pose_delay=.1,tau=.15,yaw_gain=.8,noise_xy=.02,noise_yaw=math.radians(1),jitter=True)]
    routes={'straight':[(0.,0.),(5.,0.)],'left90':[(0.,0.),(3.,0.),(3.,3.)],
            'right90':[(0.,0.),(3.,0.),(3.,-3.)],'hairpin':[(0.,0.),(3.,0.),(.1,.05)],
            'parallel':[(0.,0.),(3.,0.),(.01,.2),(-2.,.2)]}
    results=[]
    for name,points in routes.items():
        for scenario in scenarios:
            results.append(simulate(name,points,scenario,root,source,p,limits,target_speed=args.target_speed,use_reference_speed_profile=args.reference_speed_profile))
            (root/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
    (root/'configuration.json').write_text(json.dumps(dict(vehicle_parameters=p,mppi=asdict(MppiConfig(min_radius=limits.radius,max_speed=limits.max_speed,max_yaw_rate=limits.max_yaw_rate,max_yaw_accel=limits.max_yaw_accel,target_speed=args.target_speed,use_reference_speed_profile=args.reference_speed_profile))),indent=2,ensure_ascii=False))

if __name__=='__main__':main()
