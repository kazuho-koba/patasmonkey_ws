#!/usr/bin/env python3
"""ground候補N1／N3・5平均・中央値から三指標を再計算するオフライン比較。

実使用TF・採用depthを共通化し、ground gateも元の独立frameから一度だけ決める。
obstacleは従来N3解除方式の一系統を全対照で共有する。通常perceptionは変更しない。
"""
import argparse
from collections import Counter, defaultdict, deque
import csv
import hashlib
import json
from pathlib import Path
import resource
import time

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

from diagnose_height_candidates import prepare_frame
from diagnose_height_candidate_sequence import classify
from evaluate_single_depth_terrain import messages, stamp_ns
from evaluate_independent_depth_frames import pose_for_footprint
from evaluate_frame_feature_coverage import footprint_cells
from evaluate_obstacle_window_clearance import residual_maxima, write_csv
from pm_evaluation.height_candidates import CandidateOptions, extract_candidates, plane_reasons
from pm_evaluation.height_candidate_tracks import CandidateTracks
from pm_evaluation.obstacle_height_window import ObstacleHeightWindow
from pm_evaluation.ground_temporal import MODES, GroundHistory, Samples, features, seam_diagnostics
from pm_perception.mapper_replay_trace import load_mapper_trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag'); parser.add_argument('--mapper-trace',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--config',default=str(Path(__file__).resolve().parents[1]/'config/height_candidate_diagnosis.yaml'))
    parser.add_argument('--reference', help='以前のobstacle解除試験directory。raw対照全footprintを照合')
    args = parser.parse_args()
    output = Path(args.output); output.mkdir(parents=True,exist_ok=False)
    meta,frames,_ = load_mapper_trace(args.mapper_trace); params = meta['parameters']
    options = CandidateOptions(**yaml.safe_load(Path(args.config).read_text()))
    limits = np.array([params[k] for k in ('hazard_slope_limit_deg','hazard_roughness_limit','hazard_step_limit','hazard_obstacle_height_limit')])
    res = params['resolution']; modes = ('raw_frame',)+MODES
    tracker = CandidateTracks(5,.05,.03); histories = defaultdict(lambda: deque(maxlen=5))
    ground = GroundHistory(); obstacle = {}
    terrain = {mode:{} for mode in modes}; sources = {mode:{} for mode in modes}
    samples = defaultdict(lambda: defaultdict(Samples)); counters = Counter()
    rows,black_rows,frame_rows = [],[],[]; processed = []; cursor=0; path=[]
    for stamp,frame in frames.items():
        base=frame['base_to_map']; pose=pose_for_footprint(stamp,base['translation']+base['quaternion'])
        if not path or np.hypot(pose[1]-path[-1][1],pose[2]-path[-1][2])>=.25:
            path.append(pose)
    began,cpu=time.perf_counter(),time.process_time()

    def emit(pose):
        cells=footprint_cells(pose,res,.45,.55)
        for mode in modes:
            values=[list(terrain[mode].get(c,[np.nan]*3))+[obstacle[c].value if c in obstacle else np.nan] for c in cells]
            rows.append(dict(stamp_ns=pose[0],mode=mode,**classify(values,limits)))
            for cell,value in zip(cells,values):
                flags=np.isfinite(value)&(np.rint(np.clip(np.asarray(value)/limits,0,1)*100)>=100)
                if flags.any():
                    black_rows.append(dict(stamp_ns=pose[0],mode=mode,cell_x=cell[0],cell_y=cell[1],
                        causes='+'.join(name for name,flag in zip(('slope','roughness','step','obstacle'),flags) if flag),
                        slope_deg=value[0],roughness_m=value[1],step_m=value[2],obstacle_m=value[3],
                        terrain_stamp_ns=sources[mode].get(cell)))

    for _,_,record in messages(args.bag,[params['depth_topic']]):
        depth=deserialize_message(record.data,Image); stamp=stamp_ns(depth)
        if stamp not in frames:
            continue
        while cursor<len(path) and path[cursor][0]<=stamp:
            emit(path[cursor]);cursor+=1
        tick=time.perf_counter()
        points,_,u,v,pose,baseline,planes=prepare_frame(depth,frames[stamp],params)
        reasons=plane_reasons(baseline,params)
        candidates,layers=extract_candidates(points,np.column_stack((u,v)),baseline['origin'],
            baseline['hazard'].shape,res,planes,reasons['plane_usable'],limits[3],options)
        maximum=residual_maxima(points,baseline['origin'],baseline['hazard'].shape,res,planes,reasons['plane_usable'])
        origin=np.rint(baseline['origin']/res).astype(int); shape=baseline['hazard'].shape; width=shape[1]
        groups=defaultdict(list)
        for c in candidates:
            cell=(int(origin[0]+c['slot']%width),int(origin[1]+c['slot']//width))
            c['high_hit']=bool(c['max_residual_m']>=limits[3]) if c['plane_usable'] else None
            groups[cell].append(c)
        heights={mode:np.full(shape,np.nan,dtype=np.float32) for mode in MODES}
        stamps={mode:np.full((5,)+shape,-1,dtype=np.int64) for mode in MODES}
        ground_count=0
        for slot in np.flatnonzero(layers['point_count']):
            yy,xx=divmod(int(slot),width);cell=(int(origin[0]+xx),int(origin[1]+yy))
            tracked=tracker.update(cell,groups[cell],stamp,len(processed)); observed_histories=[]
            for candidate,tracking in zip(groups[cell],tracked):
                history=histories[tracking['track_id']];history.append((stamp,candidate['high_hit']))
                observed_histories.append(list(history))
                if candidate['kind']=='ground_provisional':
                    ground_count+=1
                    estimates=ground.update(tracking['track_id'],stamp,candidate['z_mean_m'])
                    for mode,(height,history_stamps) in estimates.items():
                        heights[mode][yy,xx]=height
                        stamps[mode][-len(history_stamps):,yy,xx]=history_stamps
            # 同じ基準平面・同じ候補対応を使い、obstacle側はN=3に固定する。
            height=float(maximum[slot])
            if np.isfinite(height):
                assert bool(height>=limits[3])==bool(layers['above_plane_hit'][yy,xx])
                state=obstacle.setdefault(cell,ObstacleHeightWindow(3,limits[3]))
                onset=state.onset if state.active else stamp
                confirmed=any(len(h)>=3 and all(s>=onset and value is True for s,value in h[-3:]) for h in observed_histories)
                previous=state.update(height,stamp,confirmed)
                if previous:
                    counters['obstacle_clear_'+previous]+=1
        counters['observed_cells']+=int(np.count_nonzero(layers['point_count']))
        counters['ground_candidate_observations']+=ground_count
        for mode in modes:
            feature_tick=time.perf_counter()
            evaluated=baseline if mode=='raw_frame' else features(heights[mode],params,pose[-1])
            if mode!='raw_frame':
                # maskは全方式で共通。履歴平均で過去セルを穴埋めしない。
                assert np.array_equal(np.isfinite(heights[mode]),np.isfinite(heights['ground_N1']))
                diagnostics=seam_diagnostics(heights[mode],heights['ground_N1'],stamps[mode],stamp)
                for key,value in diagnostics.items(): samples[mode][key].add(value)
                n=int(mode[-1]); sizes=diagnostics['window_size']
                counters[mode+'_full_window']+=int(np.count_nonzero(sizes==n))
                samples[mode]['input_change_vs_N1_m'].add(np.abs(heights[mode]-heights['ground_N1']))
            for key in ('slope_deg','roughness','step_height'):
                samples[mode][key].add(evaluated[key])
            valid=np.isfinite(evaluated['slope_deg'])&np.isfinite(evaluated['roughness'])&np.isfinite(evaluated['step_height'])
            for yy,xx in zip(*np.nonzero(valid)):
                cell=(int(origin[0]+xx),int(origin[1]+yy))
                terrain[mode][cell]=[float(evaluated[key][yy,xx]) for key in ('slope_deg','roughness','step_height')]
                sources[mode][cell]=stamp
            frame_rows.append(dict(stamp_ns=stamp,mode=mode,ground_cells=ground_count,
                complete_terrain_cells=int(valid.sum()),fit_cells=int(np.isfinite(evaluated['roughness']).sum()),
                roughness_mean_m=float(np.nanmean(evaluated['roughness'])) if np.isfinite(evaluated['roughness']).any() else None))
            samples[mode]['feature_ms'].add([(time.perf_counter()-feature_tick)*1000])
        processed.append(stamp);samples['shared']['frame_ms'].add([(time.perf_counter()-tick)*1000])
        if len(processed)%200==0: print('evaluated %d frames'%len(processed),flush=True)
    if processed!=list(frames): raise ValueError('採用画像列がtraceと一致しません')
    while cursor<len(path): emit(path[cursor]);cursor+=1
    results={}
    for mode in modes:
        selected=[r for r in rows if r['mode']==mode];names=('slope','roughness','step','obstacle')
        total=sum(r['cells'] for r in selected);known=sum(r['known_cells'] for r in selected)
        results[mode]=dict(footprints=len(selected),black_footprints=sum(r['black_cells']>0 for r in selected),
            black_percent=100*sum(r['black_cells']>0 for r in selected)/len(selected),cells=total,
            black_cells=sum(r['black_cells'] for r in selected),unknown_cell_percent=100*(1-known/total),
            all_cues_known_cells=sum(r['all_cues_known_cells'] for r in selected),
            fully_known_footprints=sum(r['all_cues_known_cells']==r['cells'] for r in selected),
            causes={name:sum(r[name+'_black_cells']>0 for r in selected) for name in names},
            exclusive_causes=dict(Counter('+'.join(name for name in names if r[name+'_black_cells']) for r in selected if r['black_cells'])))
    # 共通のobstacle系列であることを通過時刻ごとにも確認する。
    by_mode={m:[r for r in rows if r['mode']==m] for m in modes}
    assert all([r['obstacle_black_cells'] for r in by_mode[m]]==[r['obstacle_black_cells'] for r in by_mode['raw_frame']] for m in modes)
    reference_audit=None
    if args.reference:
        old=list(csv.DictReader((Path(args.reference)/'footprints.csv').open()))
        old=[r for r in old if r['mode']=='N3'];new=by_mode['raw_frame']
        assert len(old)==len(new)
        for before,after in zip(old,new):
            for key in after:
                if key!='mode': assert str(after[key])==before[key],(key,after[key],before[key])
        reference_audit=dict(directory=args.reference,identical_footprints=len(new),all_columns_except_mode=True)
    summary=dict(parameters=params,candidate_options=vars(options),frames=len(processed),results=results,
        counters=dict(counters),diagnostics={mode:{key:value.summary() for key,value in group.items()} for mode,group in samples.items()},
        reference_audit=reference_audit,trace_sha256=hashlib.sha256(Path(args.mapper_trace).read_bytes()).hexdigest(),
        cpu_seconds=time.process_time()-cpu,wall_seconds=time.perf_counter()-began,
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
        assumptions=['現在frameのground maskを共通化。過去セルの穴埋めなし',
                     '窓未充足では利用可能観測で平均。time TTLなし',
                     'ground gateは元の独立frameで固定し、平滑化後に候補を取り直さない',
                     'ground候補の選別biasと高さ対応gateの打切りを含む',
                     'seam診断は真の誤差ではなく隣接差と履歴不一致のproxy'])
    summary['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (Path(__file__).resolve(),Path(__file__).resolve().parents[1]/'pm_evaluation/ground_temporal.py')}
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    write_csv(output/'footprints.csv',rows);write_csv(output/'black_cells.csv',black_rows);write_csv(output/'frames.csv',frame_rows)
    print(json.dumps(results,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
