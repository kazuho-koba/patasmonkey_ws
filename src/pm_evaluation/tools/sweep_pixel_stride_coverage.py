#!/usr/bin/env python3
"""同じ画像・撮像時刻TFでstride 4/2/1の既知領域と負荷を比較する。

ROSの再生速度や受信dropを比較に混ぜず、実nodeと同じIndependentObstaclesを使用する。
unknown減少が黒増加への置換でないかも併記する。正本設定は変更しない。
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import time
import yaml
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from evaluate_single_depth_terrain import messages,stamp_ns
from evaluate_independent_depth_frames import recorded_transform,pose_for_footprint
from pm_perception.depth_projection import sampled_points,transform_points
from pm_perception.independent_obstacles import IndependentObstacles
from pm_perception.mapper_replay_trace import load_mapper_trace
from pm_planner.terrain_cost import TerrainCost


def coverage(model,pose):
    """前方1m幅の距離帯の実セルを評価。姿勢とmap原点は全stride共通。"""
    obstacle,terrain=model.costs();h,w=obstacle.shape;r=model.res
    yy,xx=np.mgrid[:h,:w];x=model.origin[0]+(xx+.5)*r-pose[1];y=model.origin[1]+(yy+.5)*r-pose[2]
    c,s=np.cos(pose[-1]),np.sin(pose[-1]);forward=x*c+y*s;lateral=-x*s+y*c
    result={}
    for distance in (1,2,3):
        mask=(forward>=distance-.5)&(forward<distance+.5)&(np.abs(lateral)<=.5)
        values=obstacle[mask]
        result.update({f'd{distance}_'+key:int(value) for key,value in
            (('cells',len(values)),('free',np.count_nonzero(values==0)),
             ('unknown',np.count_nonzero(values<0)),('black',np.count_nonzero(values>=100)))})
    d=np.arange(0.,3.501,.05);qx=pose[1]+d*c;qy=pose[2]+d*s
    ix=np.floor((qx-model.origin[0])/r).astype(int);iy=np.floor((qy-model.origin[1])/r).astype(int)
    inside=(ix>=0)&(ix<w)&(iy>=0)&(iy<h)
    raw=np.full(len(d),-1);raw[inside]=obstacle[iy[inside],ix[inside]]
    env=TerrainCost(obstacle.ravel(),w,h,r,(*model.origin,0.),.36)
    expanded=env.sample(qx,qy)
    for name,blocked in (('raw_unknown',raw<0),('footprint_blocked',expanded>=100)):
        ids=np.flatnonzero(blocked)
        result[name+'_first_m']=float(d[ids[0]]) if len(ids) else 3.5
    result['footprint_clear_to_1m']=int(np.all(expanded[d<=1.]<100))
    result['footprint_clear_to_2m']=int(np.all(expanded[d<=2.]<100))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('bag');parser.add_argument('--trace',required=True)
    parser.add_argument('--config',default='src/pm_config/config/planner_perception.yaml');parser.add_argument('--output',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    meta,frames,snapshots=load_mapper_trace(args.trace);p=meta['parameters']
    cfg=yaml.safe_load(Path(args.config).read_text())['/**']['ros__parameters']
    models={s:IndependentObstacles(p['resolution'],p['map_size_x'],p['hazard_obstacle_height_limit'],
        cfg['confirmation_window'],cfg['clear_min_points'],cfg['clear_count_ratio'],cfg['clear_ground_tolerance']) for s in (4,2,1)}
    selected={row['last_image_stamp_ns'] for row in snapshots}
    rows=[];times={s:[] for s in models};updates={s:[] for s in models};counts={s:[] for s in models};seen=[]
    for _,_,record in messages(args.bag,[p['depth_topic']]):
        image=deserialize_message(record.data,Image);stamp=stamp_ns(image)
        if stamp not in frames:continue
        frame=frames[stamp];base=frame['base_to_map'];pose=pose_for_footprint(stamp,base['translation']+base['quaternion'])
        for stride,model in models.items():
            start=time.perf_counter();points=sampled_points(image,*frame['intrinsics'],stride,p['min_depth'],p['max_depth'])
            points=transform_points(points,recorded_transform(frame['camera_to_map']))
            begin=time.perf_counter()
            model.update(points,frame['camera_to_map']['translation'],base['translation'],stamp,pose[-1],
                         p['hazard_slope_limit_deg'],p['hazard_roughness_limit'],p['hazard_step_limit'])
            end=time.perf_counter();times[stride].append((end-start)*1000);updates[stride].append((end-begin)*1000)
            counts[stride].append(len(points))
            if stamp in selected:rows.append(dict(stamp_ns=stamp,stride=stride,**coverage(model,pose)))
        seen.append(stamp)
        if len(seen)%100==0:print('frames',len(seen),flush=True)
    if seen!=list(frames):raise ValueError('採用画像列がtraceと一致しません')
    summary={}
    for stride in models:
        chosen=[row for row in rows if row['stride']==stride];result={}
        for distance in (1,2,3):
            total=sum(row[f'd{distance}_cells'] for row in chosen)
            result[f'distance_{distance}m']={name:100*sum(row[f'd{distance}_'+name] for row in chosen)/total
                                            for name in ('free','unknown','black')}
        result.update(snapshots=len(chosen),
            footprint_clear_to_1m_percent=100*np.mean([r['footprint_clear_to_1m'] for r in chosen]),
            footprint_clear_to_2m_percent=100*np.mean([r['footprint_clear_to_2m'] for r in chosen]),
            raw_unknown_first_m_median=float(np.median([r['raw_unknown_first_m'] for r in chosen])),
            footprint_blocked_first_m_median=float(np.median([r['footprint_blocked_first_m'] for r in chosen])),
            sampled_points_mean=float(np.mean(counts[stride])),
            processing_ms=dict(mean=float(np.mean(times[stride])),p95=float(np.percentile(times[stride],95))),
            model_update_ms=dict(mean=float(np.mean(updates[stride])),p95=float(np.percentile(updates[stride],95))))
        summary[str(stride)]=result
    data=dict(frames=len(seen),strides=summary,parameters=p,extra_parameters=cfg,
        trace_sha256=hashlib.sha256(Path(args.trace).read_bytes()).hexdigest())
    (out/'summary.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
    with (out/'snapshots.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps(data,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
