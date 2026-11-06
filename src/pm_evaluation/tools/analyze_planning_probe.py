#!/usr/bin/env python3
"""実MPPI入力で、障害物の寄与・unknown・速度とhorizonの影響を分離する。

no_black/no_unknownは原因切分け専用の反実仮想であり、運用するcost変更ではない。
通常nodeの計画結果を同一乱数状態から再現できたことを先に検査する。
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np
from pm_evaluation.planning_probe import restore_solver,environment
from pm_planner.perception_input import select_costs


def solve(data,values,config=None):
    solver=restore_solver(data,config)
    try:
        result=solver.step(data['pose'],data['velocity'],previous_command=data['previous'],environment=environment(data,values))
        return result,None
    except ValueError as exc:return None,str(exc)


def metrics(result,pose):
    if result is None:return dict(success=False,length=None,forward=None,lateral=None,first_v=None)
    xy=np.vstack((pose[:2],result['trajectory'][:,:2]));delta=result['trajectory'][:,:2]-pose[:2]
    c,s=np.cos(pose[2]),np.sin(pose[2])
    return dict(success=True,length=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
        forward=float(np.max(delta[:,0]*c+delta[:,1]*s)),lateral=float(np.max(np.abs(-delta[:,0]*s+delta[:,1]*c))),
        first_v=float(result['command'][0]))


def difference(first,second):
    if first is None or second is None:return None
    if first['trajectory'].shape!=second['trajectory'].shape:return None
    return float(np.max(np.linalg.norm(first['trajectory'][:,:2]-second['trajectory'][:,:2],axis=1)))


def geometry(data,values):
    h,w=data['obstacle'].shape;res=float(data['resolution']);origin=data['origin'];pose=data['pose']
    yy,xx=np.mgrid[:h,:w];x=origin[0]+(xx+.5)*res-pose[0];y=origin[1]+(yy+.5)*res-pose[1]
    c,s=np.cos(pose[2]),np.sin(pose[2]);along=x*c+y*s;lateral=-x*s+y*c
    black=values.reshape(h,w)>=100;roi=(along>=2.)&(along<=3.)
    band=np.abs(lateral)<=float(data['radius'])+res
    query=np.arange(0.,4.001,.05);qx=pose[0]+query*c;qy=pose[1]+query*s
    result=dict(black_2_3m_any=int(np.count_nonzero(black&roi)),
        black_2_3m_corridor=int(np.count_nonzero(black&roi&band)))
    for name,masked in (('black',np.where(values>=100,100,0)),('unknown',np.where(values<0,100,0))):
        # map外は各仮想gridでも危険なので、first_hitはmap境界による場合もある。
        env=environment(data,masked);hazard=env.sample(qx,qy)>=100
        result[name+'_at_pose']=bool(hazard[0])
        result[name+'_first_hit_m']=float(query[np.flatnonzero(hazard)[0]]) if hazard.any() else None
    return result


def plot_case(data,results,row,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap,BoundaryNorm
    from matplotlib.patches import Circle
    h,w=data['obstacle'].shape;res=float(data['resolution']);ox,oy=data['origin'][:2]
    extent=(ox,ox+w*res,oy,oy+h*res)
    fig,ax=plt.subplots(figsize=(8,8))
    colors=ListedColormap(['#a6a6bf','white','black']);norm=BoundaryNorm([-1.5,-.5,99.5,100.5],colors.N)
    ax.imshow(data['obstacle'],origin='lower',extent=extent,cmap=colors,norm=norm)
    ref=data['reference'];ax.plot(ref[:,0],ref[:,1],color='royalblue',label='Straight reference (8 m)')
    for name,color,style in (('baseline','green','-'),('no_black','cyan','--'),('normal','orange','-'),('no_unknown','purple','--')):
        result=results[name]
        if result is not None:
            t=np.vstack((data['pose'],result['trajectory']));ax.plot(t[:,0],t[:,1],color=color,ls=style,lw=2,label=name)
    pose=data['pose'];ax.add_patch(Circle(pose[:2],float(data['radius']),fill=False,color='red'))
    ax.arrow(pose[0],pose[1],.45*np.cos(pose[2]),.45*np.sin(pose[2]),color='red',head_width=.1)
    ax.set(xlim=extent[:2],ylim=extent[2:],xlabel='odom x [m]',ylabel='odom y [m]',aspect='equal',
        title='frame %d | black at 2-3 m corridor: %d\nno_black / no_unknown are diagnostic only'%(row['index'],row['black_2_3m_corridor']))
    ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(path,dpi=140);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('probe');parser.add_argument('--output',required=True)
    args=parser.parse_args();probe=Path(args.probe);output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    events=[json.loads(line) for line in (probe/'events.jsonl').read_text().splitlines()]
    rows=[];examples={};timing=[]
    for event in events:
        with np.load(probe/event['file']) as z:data={name:z[name] for name in z.files}
        h,w=data['obstacle'].shape
        values=select_costs(data['obstacle'].ravel(),data['terrain'].ravel(),h*w,str(data['mode']))
        assert np.array_equal(environment(data).grid,data['expanded']), '保存された実gridと復元gridが不一致'
        config=json.loads(str(data['config']));normal=dict(config,steps=25,dt=.1,target_speed=1.)
        row=dict(index=event['index'],map_stamp_ns=int(data['map_stamp_ns']),v=float(data['velocity'][0]),
                 **geometry(data,values))
        results={}
        for name,modified,settings in (
            ('baseline',values,config),('no_black',np.where(values>=100,0,values),config),
            ('no_unknown',np.where(values<0,0,values),config),('clear_all',np.zeros_like(values),config),
            ('normal',values,normal),('normal_no_black',np.where(values>=100,0,values),normal),
            ('normal_no_unknown',np.where(values<0,0,values),normal)):
            result,error=solve(data,modified,settings);results[name]=result
            row.update({name+'_'+key:value for key,value in metrics(result,data['pose']).items()})
            row[name+'_error']=error
        baseline=results['baseline']
        if event['success']:
            assert baseline is not None and np.allclose(baseline['trajectory'],data['trajectory'],rtol=0,atol=1e-10), '実計画を再現できません'
        else:assert baseline is None and row['baseline_error']==event['error'], '実失敗理由を再現できません'
        row['no_black_max_path_difference_m']=difference(baseline,results['no_black'])
        row['normal_no_black_max_path_difference_m']=difference(results['normal'],results['normal_no_black'])
        rows.append(row)
        candidates=[]
        if row['baseline_success'] and row['black_2_3m_corridor']>0:candidates.append('far_obstacle')
        if row['baseline_success'] and (row['no_black_max_path_difference_m'] or 0)>1e-6:candidates.append('obstacle_changes_path')
        if not row['baseline_success'] and row['no_unknown_success']:candidates.append('unknown_blocks')
        for name in candidates:
            if name not in examples:
                plot_case(data,results,row,output/(name+'.png'));examples[name]=row['index']
        if len(rows)%100==0:print('analyzed',len(rows),flush=True)
    stats={}
    for name in ('baseline','no_black','no_unknown','clear_all','normal','normal_no_black','normal_no_unknown'):
        good=[r for r in rows if r[name+'_success']]
        stats[name]=dict(success=len(good),errors=dict(Counter(r[name+'_error'] for r in rows if not r[name+'_success'])),
            length_m={k:float(v) for k,v in zip(('p50','p95','max'),np.percentile([r[name+'_length'] for r in good],[50,95,100]))} if good else {})
    far=[r for r in rows if r['black_2_3m_corridor']>0]
    summary=dict(events=len(rows),reproduction_verified=len(rows),variants=stats,examples=examples,
        far_corridor=dict(events=len(far),baseline_success=sum(r['baseline_success'] for r in far),
            baseline_unchanged_without_black=sum(r['baseline_success'] and r['no_black_success'] and r['no_black_max_path_difference_m']<1e-9 for r in far),
            normal_success=sum(r['normal_success'] for r in far),
            normal_changed_without_black=sum(r['normal_success'] and r['normal_no_black_success'] and r['normal_no_black_max_path_difference_m']>1e-6 for r in far)),
        baseline_changed_without_black=sum(r['baseline_success'] and r['no_black_success'] and r['no_black_max_path_difference_m']>1e-6 for r in rows),
        baseline_unknown_at_pose=sum(r['unknown_at_pose'] for r in rows))
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    with (output/'frames.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
