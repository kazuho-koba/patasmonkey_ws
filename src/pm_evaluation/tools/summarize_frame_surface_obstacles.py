#!/usr/bin/env python3
"""方式E候補のbaseline一致・極値画素・傾斜寄与を集計する。

画素近傍とのdepth差は不連続のproxyであり、外れ値の真値ラベルではない。
cell内傾斜寄与も推定地面に依存するため、説明候補として扱う。
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from evaluate_spatial_feature_coverage import aggregate, cause_summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--reference-root', type=Path, required=True)
    parser.add_argument('--radii', type=int, nargs='+', default=[2, 5, 10])
    args = parser.parse_args(); results = []
    for radius in args.radii:
        source = args.root/('radius'+str(radius))
        summary = json.loads((source/'summary.json').read_text())
        reference = json.loads((args.reference_root/('radius%d_stride4'%radius)/'summary.json').read_text())
        # hookはobstacle比較store以外を変えない。元CSV全通過行も文字列で照合。
        assert summary['path'] == reference['path']
        assert summary['path_causes'] == reference['path_causes']
        def passage_csv(path):
            with path.open() as stream:
                return [r for r in csv.DictReader(stream) if r['event'] == 'before_passage']
        assert passage_csv(source/'path_footprints.csv') == passage_csv(
            args.reference_root/('radius%d_stride4'%radius)/'path_footprints.csv')
        with (source/'black_cell_evidence.csv').open() as stream:
            rows = [{k:(int(v) if k in ('stamp_ns', 'cell_x', 'cell_y', 'count',
                      'min_u', 'min_v', 'max_u', 'max_v') else float(v))
                     for k,v in row.items()} for row in csv.DictReader(stream)]
        # 指標計算済みCSVを集計の一次入力とする。JSON書出しだけ中断した場合も
        # bagの再投影なしで再集計できる。検算済みbaselineを必ず共通参照とする。
        result = dict(radius_m=summary['radius_m'], max_depth=summary['parameters']['max_depth'],
            baseline=dict(path=summary['path']['before_passage'], causes=summary['path_causes']['before_passage']),
            modes={}, raw_black_observations=len(rows), raw_black_reclassification={})
        limit = summary['parameters']['hazard_obstacle_height_limit']
        for mode in ('quantile_span', 'detrended_span', 'surface_q90'):
            with (source/(mode+'_passages.csv')).open() as stream:
                passages = [{k:int(v) for k,v in row.items()} for row in csv.DictReader(stream)]
            assert len(passages) == 370
            result['modes'][mode] = dict(path=aggregate(passages), causes=cause_summary(passages))
            baseline_rows = passage_csv(source/'path_footprints.csv')
            transitions = dict(black_to_known_nonblack=0, black_to_wholly_unknown=0,
                               nonblack_to_black=0, retained_black=0)
            for old,new in zip(baseline_rows,passages):
                assert int(old['passage_stamp_ns']) == new['passage_stamp_ns']
                old_black = int(old['black_cells']) > 0; new_black = new['black_cells'] > 0
                if old_black and not new_black:
                    transitions['black_to_known_nonblack' if new['known_cells'] else 'black_to_wholly_unknown'] += 1
                elif old_black and new_black:
                    transitions['retained_black'] += 1
                elif not old_black and new_black:
                    transitions['nonblack_to_black'] += 1
            result['modes'][mode]['passage_transitions'] = transitions
            finite_rows = [r for r in rows if np.isfinite(r[mode])]
            result['raw_black_reclassification'][mode] = dict(known=len(finite_rows), unknown=len(rows)-len(finite_rows),
                still_black=int(sum(np.rint(np.clip(r[mode]/limit,0,1)*100) >= 100 for r in finite_rows)))
        spans = np.array([r['raw_span'] for r in rows])
        counts = np.array([r['count'] for r in rows])
        contribution = np.array([r['plane_a']*(r['max_x']-r['min_x'])+
                                 r['plane_b']*(r['max_y']-r['min_y']) for r in rows])
        finite = np.isfinite(contribution)
        # 同じセルのmin/max画素間で、fit平面が高さ差の半分以上を説明するか。
        # 他の点が残差max/minになる可能性もあるためdetrended spanも併記する。
        endpoint_jump = np.array([max(abs(r['min_depth']-r['min_depth_patch_median']),
                                     abs(r['max_depth']-r['max_depth_patch_median'])) for r in rows])
        result['baseline_verified'] = True
        result['evidence_diagnostics'] = dict(
            counts_percentiles=np.percentile(counts,[0,50,95,100]).tolist() if len(rows) else [],
            count_two=int((counts == 2).sum()), count_below_five=int((counts < 5).sum()),
            plane_known=int(finite.sum()),
            plane_slope_gt_20deg=int(sum(np.isfinite(r['plane_slope_deg']) and r['plane_slope_deg'] > 20 for r in rows)),
            plane_rms_gt_3cm=int(sum(np.isfinite(r['plane_rms']) and r['plane_rms'] > .03 for r in rows)),
            plane_explains_half_span=int((finite & (contribution >= spans*.5)).sum()),
            extrema_depth_equal_within_1mm=sum(abs(r['max_depth']-r['min_depth']) <= .001 for r in rows),
            extrema_pixel_vertical_gap_gt_100=sum(abs(r['max_v']-r['min_v']) > 100 for r in rows),
            endpoint_depth_patch_jump_gt_5cm=int((endpoint_jump > .05).sum()),
            note='frame-cell更新数。近傍depth差は境界・傾斜でも生じ、外れ値の確定ではない。')
        # 個別確認用：plane差引で非黒になる例、分位点だけで非黒になる例、未解消例。
        limit = summary['parameters']['hazard_obstacle_height_limit']*.995
        groups = dict(plane_reduced=[r for r in rows if np.isfinite(r['detrended_span']) and r['detrended_span'] < limit],
                      quantile_reduced=[r for r in rows if r['quantile_span'] < limit],
                      plane_still_black=[r for r in rows if r['detrended_span'] >= limit])
        result['examples'] = {name: sorted(group, key=lambda r:r['raw_span'], reverse=True)[:3]
                              for name,group in groups.items()}
        results.append(result)
    (args.root/'analysis.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    for result in results:
        print('radius',result['radius_m'], 'diagnostics',result['evidence_diagnostics'])
        for mode,data in [('baseline',result['baseline'])]+list(result['modes'].items()):
            print(mode,data['path']['black_footprints'],data['path']['black_percent_of_known'],
                  data['path']['unknown_cell_percent'],data['causes']['overlapping_cue_footprints'])
        print('raw black reclassification',result['raw_black_reclassification'])


if __name__ == '__main__':
    main()
