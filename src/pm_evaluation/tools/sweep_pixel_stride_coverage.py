#!/usr/bin/env python3
"""同じ画像・撮像時刻TFでstride 4/2/1の既知領域と負荷を比較する。

ROSの再生速度や受信dropを比較に混ぜず、実nodeと同じIndependentObstaclesを使用する。
unknown減少が黒増加への置換でないかも併記する。正本設定は変更しない。
"""
import argparse
import csv
import hashlib
import inspect
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


BANDS={d:(d-.5, d+.5 if d<6 else 6.) for d in range(1,7)}


def coverage(model,pose):
    """同じodomセル中心を母集団とし、map外もunknownとして数える。

    body_alignedの外接矩形の角を分母へ加えず、legacyのmap外を除外もしない。
    距離帯は車体前方・横幅1m。6m帯だけ保持端までの5.5〜6.0mとする。
    """
    obstacle,terrain=model.costs();h,w=obstacle.shape;r=model.res
    c,s=np.cos(pose[-1]),np.sin(pose[-1])
    # 評価領域を包むodom軸のセルを列挙する。この母集団はmodelの原点・yaw別の
    # 配列寸法に依存しないため、保持方式間でも同じ地点を比較できる。
    corners=np.array([[.5,-.5],[6.,-.5],[6.,.5],[.5,.5]])@np.array([[c,s],[-s,c]])+np.asarray(pose[1:3])
    lower=np.floor(corners.min(axis=0)/r).astype(int);upper=np.ceil(corners.max(axis=0)/r).astype(int)
    yy,xx=np.mgrid[lower[1]:upper[1],lower[0]:upper[0]]
    x=(xx+.5)*r-pose[1];y=(yy+.5)*r-pose[2]
    forward=x*c+y*s;lateral=-x*s+y*c
    ix=xx-int(round(model.origin[0]/r));iy=yy-int(round(model.origin[1]/r))
    inside=(ix>=0)&(ix<w)&(iy>=0)&(iy<h)
    sampled=np.full(ix.shape,-1,dtype=np.int8);sampled[inside]=obstacle[iy[inside],ix[inside]]
    result={}
    for distance,(start,end) in BANDS.items():
        mask=(forward>=start)&(forward<end)&(np.abs(lateral)<=.5)
        values=sampled[mask]
        result.update({f'd{distance}_'+key:int(value) for key,value in
            (('cells',len(values)),('free',np.count_nonzero(values==0)),
             ('unknown',np.count_nonzero(values<0)),('black',np.count_nonzero(values>=100)))})
    d=np.arange(0.,3.501,.05);qx=pose[1]+d*c;qy=pose[2]+d*s
    ix=np.floor((qx-model.origin[0])/r).astype(int);iy=np.floor((qy-model.origin[1])/r).astype(int)
    inside=(ix>=0)&(ix<w)&(iy>=0)&(iy<h)
    raw=np.full(len(d),-1);raw[inside]=obstacle[iy[inside],ix[inside]]
    env=TerrainCost(obstacle.ravel(),w,h,r,(*model.origin,0.),.3,margin_m=.1)
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
    parser.add_argument('--retention-mode',choices=('body_aligned','odom_aligned'))
    parser.add_argument('--forward-offset-m',type=float)
    parser.add_argument('--compare-legacy',action='store_true',help='同じ画像から従来の保持方式も再計算')
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    meta,frames,snapshots=load_mapper_trace(args.trace);p=meta['parameters']
    cfg=yaml.safe_load(Path(args.config).read_text())['/**']['ros__parameters']
    mode=args.retention_mode or cfg.get('map_retention_mode','odom_aligned')
    offset=args.forward_offset_m if args.forward_offset_m is not None else cfg.get('map_forward_offset_m',0.)
    modes=list(dict.fromkeys([mode]+(['odom_aligned'] if args.compare_legacy else [])))
    models={(m,s):IndependentObstacles(p['resolution'],p['map_size_x'],p['hazard_obstacle_height_limit'],
        cfg['confirmation_window'],cfg['clear_min_points'],cfg['clear_count_ratio'],cfg['clear_ground_tolerance'],
        retention_mode=m,forward_offset_m=offset) for m in modes for s in (4,2,1)}
    selected={row['last_image_stamp_ns'] for row in snapshots}
    rows=[];times={s:[] for s in models};updates={s:[] for s in models};counts={s:[] for s in models};seen=[]
    for _,_,record in messages(args.bag,[p['depth_topic']]):
        image=deserialize_message(record.data,Image);stamp=stamp_ns(image)
        if stamp not in frames:continue
        frame=frames[stamp];base=frame['base_to_map'];pose=pose_for_footprint(stamp,base['translation']+base['quaternion'])
        # 同一strideの投影は保持方式間で共用する。画像、TF、評価時刻の差を作らない。
        for stride in (4,2,1):
            start=time.perf_counter();points=sampled_points(image,*frame['intrinsics'],stride,p['min_depth'],p['max_depth'])
            points=transform_points(points,recorded_transform(frame['camera_to_map']))
            projection_ms=(time.perf_counter()-start)*1000
            for m in modes:
                key=(m,stride);model=models[key];begin=time.perf_counter()
                model.update(points,frame['camera_to_map']['translation'],base['translation'],stamp,pose[-1],
                             p['hazard_slope_limit_deg'],p['hazard_roughness_limit'],p['hazard_step_limit'])
                elapsed=(time.perf_counter()-begin)*1000
                times[key].append(projection_ms+elapsed);updates[key].append(elapsed)
                counts[key].append(len(points))
                if stamp in selected:rows.append(dict(stamp_ns=stamp,retention_mode=m,stride=stride,**coverage(model,pose)))
        seen.append(stamp)
        if len(seen)%100==0:print('frames',len(seen),flush=True)
    if seen!=list(frames):raise ValueError('採用画像列がtraceと一致しません')
    if len(selected)!=len(snapshots):raise ValueError('snapshotの採用画像stampが重複しています')
    if len(rows)!=len(snapshots)*len(models):raise ValueError('評価件数が条件間で一致しません')
    # modeごとに分母が変わっていないことも確認し、map外除外による見かけの改善を防ぐ。
    denominators={}
    for row in rows:
        key=row['stamp_ns'];value=tuple(row[f'd{d}_cells'] for d in BANDS)
        if key in denominators and denominators[key]!=value:raise ValueError('距離帯の母集団が条件間で不一致')
        denominators[key]=value
    summary={}
    for key in models:
        m,stride=key
        chosen=[row for row in rows if row['stride']==stride and row['retention_mode']==m];result={}
        for distance in BANDS:
            total=sum(row[f'd{distance}_cells'] for row in chosen)
            result[f'distance_{distance}m']={name:100*sum(row[f'd{distance}_'+name] for row in chosen)/total
                                            for name in ('free','unknown','black')}
        result.update(snapshots=len(chosen),
            footprint_clear_to_1m_percent=100*np.mean([r['footprint_clear_to_1m'] for r in chosen]),
            footprint_clear_to_2m_percent=100*np.mean([r['footprint_clear_to_2m'] for r in chosen]),
            raw_unknown_first_m_median=float(np.median([r['raw_unknown_first_m'] for r in chosen])),
            footprint_blocked_first_m_median=float(np.median([r['footprint_blocked_first_m'] for r in chosen])),
            sampled_points_mean=float(np.mean(counts[key])),
            processing_ms=dict(mean=float(np.mean(times[key])),p95=float(np.percentile(times[key],95))),
            model_update_ms=dict(mean=float(np.mean(updates[key])),p95=float(np.percentile(updates[key],95))))
        summary.setdefault(m,{})[str(stride)]=result
    data=dict(frames=len(seen),strides=summary[mode],retention_modes=summary,parameters=p,extra_parameters=cfg,
        effective_retention_mode=mode,effective_forward_offset_m=offset,distance_bands_m=BANDS,
        footprint_radius_m=.3,footprint_margin_m=.1,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        model_sha256=hashlib.sha256(Path(inspect.getfile(IndependentObstacles)).read_bytes()).hexdigest(),
        image_stamps_sha256=hashlib.sha256(json.dumps(seen).encode()).hexdigest(),
        trace_sha256=hashlib.sha256(Path(args.trace).read_bytes()).hexdigest())
    (out/'summary.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
    with (out/'snapshots.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps(data,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
