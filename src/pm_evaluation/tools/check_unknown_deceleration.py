#!/usr/bin/env python3
"""unknownが1/2m先にある場合の速度計画を、既知free対照と比較する。

実車へ指令しない。人工境界と、保存済み実入力を使う。計画不成立を「滑らかに
減速できた」と数えず、現在速度・最初の指令・horizon末尾の速度を分離する。
"""
import argparse
from dataclasses import asdict
import csv
import json
from pathlib import Path
import numpy as np
from pm_planner.mppi_poc import MppiPoc,MppiConfig
from pm_planner.terrain_cost import TerrainCost
from pm_evaluation.planning_probe import restore_solver,environment
from pm_planner.perception_input import select_costs


def plan(solver,pose,velocity,env):
    try:
        result=solver.step(pose,velocity,previous_command=velocity,environment=env)
    except ValueError as exc:return None,str(exc)
    xy=np.vstack((np.asarray(pose[:2]),result['trajectory'][:,:2]))
    assert np.all(env.segment(xy[:-1,0],xy[:-1,1],xy[1:,0],xy[1:,1])<100)
    return result,None


def metrics(result):
    if result is None:return dict(first_v=None,end_v=None,min_v=None)
    controls=result['trajectory_controls']
    return dict(first_v=float(controls[0,0]),end_v=float(controls[-1,0]),
                min_v=float(controls[:,0].min()))


def first_unknown(data,values):
    h,w=data['obstacle'].shape;pose=data['pose'];d=np.arange(0.,3.501,.05)
    x=pose[0]+d*np.cos(pose[2]);y=pose[1]+d*np.sin(pose[2]);origin=data['origin'];res=float(data['resolution'])
    ix=np.floor((x-origin[0])/res).astype(int);iy=np.floor((y-origin[1])/res).astype(int)
    inside=(ix>=0)&(ix<w)&(iy>=0)&(iy<h);raw=np.full(len(d),-1)
    raw[inside]=values.reshape(h,w)[iy[inside],ix[inside]]
    ids=np.flatnonzero(raw<0);first=float(d[ids[0]]) if len(ids) else None
    only_unknown=np.where(values<0,100,0);expanded=environment(data,only_unknown)
    ids=np.flatnonzero(expanded.sample(x,y)>=100)
    inflated=float(d[ids[0]]) if len(ids) else None
    return first,inflated


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--probe',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    synthetic=[];curves=[]
    for initial in (0.,.3,.5,1.):
        for warm in (False,True):
            cfg=MppiConfig(target_speed=1.,steps=25,dt=.1,tracking_mode='corridor',path_tolerance_m=.6,min_radius=.4)
            free=np.zeros((160,160),dtype=np.int16);free_env=TerrainCost(free.ravel(),160,160,.1,(-4.,-8.,0.),.36)
            base=MppiPoc(cfg);base.set_path([(0.,0.),(8.,0.)])
            if warm:
                for _ in range(10):base.step((0.,0.,0.),(initial,0.),previous_command=(initial,0.),environment=free_env)
            rng=base.rng.bit_generator.state;nominal=base.nominal.copy();station=base.station
            for distance in (None,1.,2.):
                raw=free.copy()
                if distance is not None:raw[:,int(round((distance+4.)/.1)):]=-1
                env=TerrainCost(raw.ravel(),160,160,.1,(-4.,-8.,0.),.36)
                solver=MppiPoc(cfg);solver.set_path([(0.,0.),(8.,0.)]);solver.nominal=nominal.copy()
                solver.rng.bit_generator.state=rng;solver.station=station
                result,error=plan(solver,(0.,0.,0.),(initial,0.),env)
                row=dict(initial_v=initial,warm=warm,unknown_distance=distance,success=result is not None,error=error,**metrics(result))
                if result:
                    row['forward_m']=float(result['trajectory'][:,0].max())
                    row['end_stopping_distance_m']=row['end_v']**2/(2*.4)
                    d=np.arange(0.,4.001,.01);bad=env.sample(d,np.zeros_like(d))>=100
                    row['inflated_boundary_m']=float(d[np.flatnonzero(bad)[0]]) if bad.any() else None
                    row['remaining_center_clearance_m']=None if distance is None else row['inflated_boundary_m']-result['trajectory'][-1,0]
                    if not warm and initial in (.3,1.):curves.append((row,result))
                synthetic.append(row)
    for row in synthetic:
        control=next(r for r in synthetic if r['initial_v']==row['initial_v'] and r['warm']==row['warm'] and r['unknown_distance'] is None)
        row['first_v_delta_vs_free']=row['first_v']-control['first_v'] if row['success'] else None
        row['end_v_delta_vs_free']=row['end_v']-control['end_v'] if row['success'] else None

    real=[];probe=Path(args.probe)
    for event in map(json.loads,(probe/'events.jsonl').read_text().splitlines()):
        with np.load(probe/event['file']) as z:data={k:z[k] for k in z.files}
        h,w=data['obstacle'].shape;values=select_costs(data['obstacle'].ravel(),data['terrain'].ravel(),h*w,str(data['mode']))
        raw,expanded=first_unknown(data,values)
        distance=1 if raw is not None and .8<=raw<=1.2 else 2 if raw is not None and 1.8<=raw<=2.2 else None
        if distance is None:continue
        actual,error=plan(restore_solver(data),data['pose'],data['velocity'],environment(data,values))
        no_unknown,_=plan(restore_solver(data),data['pose'],data['velocity'],environment(data,np.where(values<0,0,values)))
        a=metrics(actual);b=metrics(no_unknown)
        real.append(dict(index=event['index'],distance_bin=distance,raw_boundary_m=raw,
            inflated_boundary_m=expanded,unknown_at_footprint=expanded==0.,initial_v=float(data['velocity'][0]),
            success=actual is not None,no_unknown_success=no_unknown is not None,error=error,
            actual_first_v=a['first_v'],actual_end_v=a['end_v'],
            first_v_delta_vs_no_unknown=a['first_v']-b['first_v'] if actual and no_unknown else None,
            end_v_delta_vs_no_unknown=a['end_v']-b['end_v'] if actual and no_unknown else None))
    summary={}
    for distance in (1,2):
        chosen=[r for r in real if r['distance_bin']==distance]
        summary[str(distance)]=dict(events=len(chosen),success=sum(r['success'] for r in chosen),
            unknown_at_current_footprint=sum(r['unknown_at_footprint'] for r in chosen),
            restored_without_unknown=sum(not r['success'] and r['no_unknown_success'] for r in chosen),
            first_command_lower=sum(r['first_v_delta_vs_no_unknown'] is not None and r['first_v_delta_vs_no_unknown']<-.01 for r in chosen),
            tail_speed_lower=sum(r['end_v_delta_vs_no_unknown'] is not None and r['end_v_delta_vs_no_unknown']<-.01 for r in chosen))
    (out/'summary.json').write_text(json.dumps(dict(synthetic=synthetic,real_summary=summary),ensure_ascii=False,indent=2))
    with (out/'real_cases.csv').open('w') as stream:
        if real:
            writer=csv.DictWriter(stream,fieldnames=list(real[0]));writer.writeheader();writer.writerows(real)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for initial,ax in zip((.3,1.),axes):
        for row,result in curves:
            if row['initial_v']!=initial:continue
            ax.plot(result['trajectory'][:,0],result['trajectory_controls'][:,0],label='free' if row['unknown_distance'] is None else 'unknown from %.0fm'%row['unknown_distance'])
        ax.set(xlabel='Forward x [m]',ylabel='Planned speed [m/s]',title='Initial speed %.1f m/s'%initial)
        ax.legend();ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(out/'speed_profiles.png',dpi=150);plt.close(fig)
    print(json.dumps(dict(synthetic=synthetic,real_summary=summary),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
