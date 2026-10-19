#!/usr/bin/env python3
"""地形短期窓比較のCSVを照合し、N1との対比較と説明図を保存する。"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def summarize(directory, repeat_reference=None):
    """率だけでなく同じ通過stampの組を照合。obstacleの差は許容しない。"""
    summary=json.loads((directory/'summary.json').read_text())
    rows=list(csv.DictReader((directory/'footprints.csv').open()))
    frame_rows=list(csv.DictReader((directory/'frames.csv').open()))
    modes=list(summary['results']);by_mode={m:[r for r in rows if r['mode']==m] for m in modes}
    base=by_mode['ground_N1'];raw=by_mode['raw_frame'];paired={}
    for mode,selected in by_mode.items():
        assert len(selected)==len(base)==summary['results']['ground_N1']['footprints']
        assert [r['stamp_ns'] for r in selected]==[r['stamp_ns'] for r in base]
        assert [r['obstacle_black_cells'] for r in selected]==[r['obstacle_black_cells'] for r in raw]
        before={r['stamp_ns'] for r in base if int(r['black_cells'])>0}
        after={r['stamp_ns'] for r in selected if int(r['black_cells'])>0}
        paired[mode]=dict(new_black_footprints=len(after-before),removed_black_footprints=len(before-after),
            new_black_stamps=sorted(after-before),removed_black_stamps=sorted(before-after))
        if mode!='raw_frame':
            # 方式間で評価可能maskが変わって黒率が変化したのではないことを確認。
            baseline=[r['complete_terrain_cells'] for r in frame_rows if r['mode']=='ground_N1']
            actual=[r['complete_terrain_cells'] for r in frame_rows if r['mode']==mode]
            assert actual==baseline
            assert [r['all_cues_known_cells'] for r in selected]==[r['all_cues_known_cells'] for r in base]
    repeat=None
    if repeat_reference:
        old=json.loads((repeat_reference/'summary.json').read_text())
        assert old['results']==summary['results'] and old['counters']==summary['counters']
        repeat=dict(directory=str(repeat_reference),identical_results_and_counters=True)
    audit=dict(paired_vs_ground_N1=paired,obstacle_identical_per_footprint=True,
        ground_modes_identical_complete_terrain_mask_per_frame=True,repeat_audit=repeat)
    (directory/'audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')
    labels=['raw','G N1','mean3','mean5','median3','median5']
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    specs=[('Black footprints [%]',[summary['results'][m]['black_percent'] for m in modes]),
           ('All 4 cues known / footprint cells [%]',[100*summary['results'][m]['all_cues_known_cells']/summary['results'][m]['cells'] for m in modes]),
           ('Local plane residual RMS: pooled p95 [mm]',[1000*summary['diagnostics'][m]['roughness']['p95'] for m in modes]),
           ('Step cue: pooled p95 [cm]',[100*summary['diagnostics'][m]['step_height']['p95'] for m in modes])]
    for ax,(title,values) in zip(axes.flat,specs):
        ax.bar(np.arange(len(modes)),values);ax.set_xticks(np.arange(len(modes)));ax.set_xticklabels(labels)
        ax.set_title(title);ax.grid(axis='y',alpha=.3)
        for i,value in enumerate(values):ax.text(i,value,'%.2f'%value,ha='center',va='bottom',fontsize=8)
    fig.tight_layout();fig.savefig(directory/'comparison.png',dpi=140);plt.close(fig)
    return audit


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory');parser.add_argument('--repeat-reference')
    args=parser.parse_args()
    print(json.dumps(summarize(Path(args.directory),Path(args.repeat_reference) if args.repeat_reference else None),ensure_ascii=False,indent=2))
