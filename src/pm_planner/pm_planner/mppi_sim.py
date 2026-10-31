"""MPPI PoCを点車体modelで閉ループ評価する。ROS・実車へは接続しない。"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
from .mppi_poc import MppiConfig, MppiPoc
from .reference import Limits, build_reference
from .progress import Progress


def save_svg(path, reference, rows):
    """参照Pathと閉ループ軌跡を同じ縮尺で保存し、地図・障害物とは区別する。"""
    ref=np.asarray(reference,dtype=float);trace=np.asarray([(r[1],r[2]) for r in rows],dtype=float)
    xy=np.vstack((ref,trace)) if len(trace) else ref
    low=xy.min(axis=0);high=xy.max(axis=0);span=np.maximum(high-low,1.)
    scale=min(800./span[0],600./span[1])
    def polyline(points,color):
        coords=' '.join('%.2f,%.2f'%(50+(p[0]-low[0])*scale,680-(p[1]-low[1])*scale) for p in points)
        return '<polyline points="%s" fill="none" stroke="%s" stroke-width="2"/>'%(coords,color)
    lines=['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="720">',
           '<rect width="900" height="720" fill="white"/>',
           '<text x="15" y="25" font-size="16">Non-driving simulation: blue reference / orange simulated trajectory</text>',
           polyline(ref,'#1575cc')]
    if len(trace):lines.append(polyline(trace,'#e77118'))
    lines.append('</svg>');path.write_text('\n'.join(lines)+'\n')


def simulate(name, points, output, config, max_steps=1200):
    """各stepで予測指令を同じ車体運動modelへ戻し、追従を実際に閉ループさせる。"""
    wp=[dict(waypoint_id=str(i),attribute=('start' if i==0 else 'goal' if i==len(points)-1 else 'pass'),
             pause_sec=0.,note='') for i in range(len(points))]
    # 通常previewの既定と同じ0.1 m間隔で参照Pathを生成する。
    ref=build_reference(points,wp,Limits(config.min_radius,config.max_speed,config.max_yaw_rate,config.max_yaw_accel),spacing=.1)
    solver=MppiPoc(config);solver.set_path([(p[0],p[1]) for p in ref.samples])
    x,y,yaw=ref.samples[0][:3];v=w=0.;rows=[];timings=[];failure='';goal=False;max_error=0.
    # 実行管理への接続はまだ行わないが、既存進捗判定との到達条件を照合する。
    progress=Progress(ref);progress.start((0.,x,y,yaw,0.))
    for i in range(max_steps):
        started=time.perf_counter()
        try:result=solver.step((x,y,yaw),(v,w))
        except ValueError as exc:failure=str(exc);break
        timings.append((time.perf_counter()-started)*1000.)
        # 行はstep開始時刻・現在pose・その時点で計算した指令を組にする。
        # 次のposeへの積分は記録後に行い、時刻と座標を1周期ずらさない。
        v,w=result['command']
        # 補間点間隔による見かけの誤差を避け、各線分への最近傍距離を求める。
        # ただし経路全体に対する距離なので、自己交差付近の正しい区間対応は保証しない。
        starts=solver.path[:-1];vectors=solver.path[1:]-starts;position=np.array([x,y])
        fraction=np.clip(np.sum((position-starts)*vectors,axis=1)/np.sum(vectors*vectors,axis=1),0.,1.)
        error=float(np.min(np.linalg.norm(starts+fraction[:,None]*vectors-position,axis=1)))
        max_error=max(max_error,error)
        rows.append((i*config.dt,x,y,yaw,v,w,result['station'],error))
        yaw+=w*config.dt;x+=v*math.cos(yaw)*config.dt;y+=v*math.sin(yaw)*config.dt
        progress.update(((i+1)*config.dt,x,y,yaw,v))
        if result['goal']:goal=True;break
    output.mkdir(parents=True,exist_ok=True)
    with (output/(name+'.csv')).open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['time_s','x_m','y_m','yaw_rad','v_mps','w_radps','station_m','nearest_error_m']);writer.writerows(rows)
    save_svg(output/(name+'.svg'),solver.path,rows)
    summary=dict(case=name,goal=goal,failure=failure,steps=len(rows),duration_s=len(rows)*config.dt,
        max_nearest_error_m=max_error,station_m=solver.station,path_length_m=float(solver.s[-1]),
        compute_ms_p50=float(np.percentile(timings,50)) if timings else None,
        compute_ms_p95=float(np.percentile(timings,95)) if timings else None,
        compute_ms_max=max(timings) if timings else None,deadline_misses=sum(t>config.dt*1000 for t in timings),
        progress_state=progress.state,progress_reason=progress.reason,
        waypoints_completed=len(progress.completed),waypoints_total=len(ref.events))
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    parser.add_argument('--batch-size',type=int,default=384);parser.add_argument('--steps',type=int,default=25)
    args=parser.parse_args();output=Path(args.output)
    if output.exists():raise SystemExit('出力先は未作成のdirectoryを指定してください')
    config=MppiConfig(batch_size=args.batch_size,steps=args.steps)
    cases={'straight':[(0.,0.),(5.,0.)], 'corner':[(0.,0.),(3.,0.),(3.,3.)],
           'turn':[(0.,0.),(3.,0.),(.1,.05)],
           'parallel_return':[(0.,0.),(3.,0.),(.01,.2),(-2.,.2)]}
    results=[simulate(name,points,output,config) for name,points in cases.items()]
    (output/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(results,ensure_ascii=False,indent=2))
    if not all(item['goal'] and item['progress_state']=='COMPLETE' for item in results):raise SystemExit(2)
