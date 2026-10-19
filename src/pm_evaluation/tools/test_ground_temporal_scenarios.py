#!/usr/bin/env python3
"""既知の人工路面で平坦化・段差保持・未知の基準ずれへの副作用を調べる。

実センサの性能試験ではない。ground対応が既知のoracle対照と、既存の
plane gate／候補抽出／対応を通すgated系を分け、選別によるunknownを隠さない。
"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from pm_evaluation.ground_temporal import GroundHistory, MODES, Samples, features, seam_diagnostics
from pm_evaluation.height_candidates import CandidateOptions, extract_candidates, plane_reasons
from pm_evaluation.height_candidate_tracks import CandidateTracks


PARAMETERS=dict(resolution=.1,feature_max_observation_age=3.,feature_neighborhood_radius_cells=1,
    feature_min_neighbors=5,hazard_slope_limit_deg=20.,hazard_roughness_limit=.03,
    hazard_step_limit=.07,step_min_side_neighbors=2)


def scenarios(seed=20261017):
    """noiseは全手法へ同一入力。case間は独立seedで再現可能にする。"""
    x,y=np.meshgrid((np.arange(21)-10)*.1,(np.arange(21)-10)*.1)
    names=('flat_jitter','slope10_jitter','step07_clean','step10_clean','step10_jitter',
           'common_offset','masked_bias_seam','mixed_surface_change')
    for k,name in enumerate(names):
        rng=np.random.RandomState(seed+k); sequence=[]
        for i in range(36):
            truth=np.zeros_like(x)
            if name=='slope10_jitter':truth=np.tan(np.deg2rad(10))*x
            if name.startswith('step07'):truth=np.where(x>=0,.07,0.)
            if name.startswith('step10'):truth=np.where(x>=0,.10,0.)
            if name=='mixed_surface_change' and i>=10:truth=np.where(x>=0,.10,0.)
            observed=truth.copy();mask=np.ones_like(x,dtype=bool)
            if name.endswith('jitter'):observed+=rng.normal(0,.015,x.shape)
            if name=='common_offset':observed+=rng.normal(0,.03)
            if name=='masked_bias_seam':
                if i<12:mask=x<=0
                elif i<24:mask=x>=0
                if i>=12:observed+=.04
            observed[~mask]=np.nan
            sequence.append((truth,observed))
        yield name,x,y,sequence


def run(output):
    """現在mask・初期gateを各方式で共通化し、heightだけ変えてfeatureを計算。"""
    output.mkdir(parents=True,exist_ok=False);results={}
    limits=np.array([20.,.03,.07]);names=('raw',)+MODES
    for scenario,x,y,sequence in scenarios():
        case={};last={}
        # 最終frameでは継ぎ目や応答遅れが解消しているため、stress例だけ途中を図化。
        plot_index=24 if scenario=='masked_bias_seam' else 11 if scenario=='mixed_surface_change' else len(sequence)-1
        for branch in ('oracle','gated'):
            history=GroundHistory();tracker=CandidateTracks(5,.05,.03)
            samples=defaultdict(lambda:defaultdict(Samples));timelines=[]
            for index,(truth,observed) in enumerate(sequence):
                stamp=index*100000000
                raw=features(observed,PARAMETERS,0.)
                fields={m:np.full_like(x,np.nan,dtype=np.float32) for m in MODES}
                stamps={m:np.full((5,)+x.shape,-1,dtype=np.int64) for m in MODES}
                selected=[]
                if branch=='oracle':
                    # 真のground表面が既知という楽観対照。入力切替stress caseでは
                    # 意図的に同じIDを使い、誤対応時の遅れを可視化する。
                    selected=[(int(yy*x.shape[1]+xx),yy,xx,float(observed[yy,xx]))
                              for yy,xx in zip(*np.nonzero(np.isfinite(observed)))]
                else:
                    baseline=dict(raw,relative_elevation=observed,
                                  age_seconds=np.where(np.isfinite(observed),0.,np.nan))
                    reasons=plane_reasons(baseline,PARAMETERS)
                    planes=np.stack((raw['plane_a'],raw['plane_b'],raw['plane_c']),axis=-1)
                    mask=np.isfinite(observed)
                    points=np.column_stack((x[mask],y[mask],observed[mask]))
                    candidates,_=extract_candidates(points,np.zeros((len(points),2)),np.array([-1.05,-1.05]),
                        x.shape,.1,planes,reasons['plane_usable'],.08,CandidateOptions())
                    groups=defaultdict(list)
                    for c in candidates:
                        c['high_hit']=bool(c['max_residual_m']>=.08) if c['plane_usable'] else None
                        groups[c['slot']].append(c)
                    for slot,rows in groups.items():
                        tracks=tracker.update((slot%21,slot//21),rows,stamp,index)
                        for c,t in zip(rows,tracks):
                            if c['kind']=='ground_provisional':
                                selected.append((t['track_id'],slot//21,slot%21,c['z_mean_m']))
                for track_id,yy,xx,z in selected:
                    for mode,(estimate,ss) in history.update(track_id,stamp,z).items():
                        fields[mode][yy,xx]=estimate;stamps[mode][-len(ss):,yy,xx]=ss
                for mode in names:
                    heights=observed if mode=='raw' else fields[mode]
                    f=raw if mode=='raw' else features(heights,PARAMETERS,0.)
                    cues=np.stack([f[k] for k in ('slope_deg','roughness','step_height')],axis=-1)
                    valid=np.isfinite(cues).all(axis=-1)
                    black=(np.isfinite(cues)&(np.rint(np.clip(cues/limits,0,1)*100)>=100)).any(axis=-1)
                    boundary=(np.abs(x)<=.3)&(np.abs(y)<=.7)
                    low=(x<-.3)&(np.abs(y)<=.7);high=(x>.3)&(np.abs(y)<=.7)
                    estimate=float(np.nanmean(heights[high])-np.nanmean(heights[low])) if np.isfinite(heights[high]).any() and np.isfinite(heights[low]).any() else None
                    timelines.append(dict(frame=index,mode=mode,step_estimate_m=estimate,
                        valid_boundary_cells=int((valid&boundary).sum()),boundary_cells=int(boundary.sum()),
                        black_boundary_cells=int((black&boundary).sum())))
                    if index>=5:
                        samples[mode]['abs_height_error_m'].add(np.abs(heights-truth))
                        samples[mode]['roughness_m'].add(f['roughness'])
                        samples[mode]['step_cue_m'].add(f['step_height'])
                        samples[mode]['slope_deg'].add(f['slope_deg'])
                        samples[mode]['complete_terrain'].add(valid.astype(float))
                        samples[mode]['terrain_black'].add(black.astype(float))
                        if estimate is not None:samples[mode]['plateau_height_difference_m'].add([estimate])
                        if mode!='raw':
                            for key,value in seam_diagnostics(heights,fields['ground_N1'],stamps[mode],stamp).items():
                                samples[mode][key].add(value)
                    if index==plot_index:
                        last[(branch,mode)]=(heights.copy(),f)
            metrics={mode:{key:sample.summary() for key,sample in group.items()} for mode,group in samples.items()}
            last_rows=[row for row in timelines if row['frame']==len(sequence)-1]
            first_detect={}
            for mode in names:
                detections=[r['frame'] for r in timelines if r['mode']==mode and r['frame']>=10
                            and r['step_estimate_m'] is not None and r['step_estimate_m']>=.07]
                first_detect[mode]=min(detections)-10 if detections else None
            case[branch]=dict(metrics=metrics,last_frame=last_rows,plot_frame_index=plot_index,
                change_test_delay_frames=first_detect if scenario=='mixed_surface_change' else None,
                timeline=timelines)
        fig,axes=plt.subplots(1,2,figsize=(12,4))
        for axis,branch in zip(axes,('oracle','gated')):
            axis.plot(x[10],sequence[plot_index][0][10],'k--',label='truth')
            for mode in names:
                axis.plot(x[10],last[(branch,mode)][0][10],label=mode)
            axis.set_title(scenario+' / '+branch+' / frame %d'%plot_index);axis.set_xlabel('x [m]');axis.set_ylabel('z [m]');axis.grid()
        axes[-1].legend(fontsize=8);fig.tight_layout();fig.savefig(output/(scenario+'.png'),dpi=140);plt.close(fig)
        results[scenario]=case
    (output/'summary.json').write_text(json.dumps(dict(parameters=PARAMETERS,seed=20261017,results=results,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (Path(__file__).resolve(),Path(__file__).resolve().parents[1]/'pm_evaluation/ground_temporal.py')},
        limitations=['人工高さgridであり実stereo誤差の再現ではない',
                     'oracleは候補選別なし。mixed_surface_changeは意図的な誤対応stress',
                     'gatedは既存ground gateによる未知化も含む',
                     'terrainだけを評価しobstacleは含めない']),ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True)
    run(Path(p.parse_args().output))
