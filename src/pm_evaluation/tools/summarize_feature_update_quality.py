"""更新遷移を対照・同一セル層別対照と比較し、通過時の黒episodeへ対応付ける。"""
import argparse
import bisect
import csv
import itertools
import json
from collections import defaultdict, Counter
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('assets', type=Path)
parser.add_argument('--mapper-trace', type=Path, required=True)
parser.add_argument('--reference-root', type=Path, required=True,
                    help='同条件で実施した通常空間評価のassets')
args = parser.parse_args()
ROOT = args.assets
FLAGS = ['farther', 'geometric_farther', 'fewer_pixels', 'fewer_support', 'worse_geometry', 'more_one_sided', 'less_side_support']
TRACE = args.mapper_trace
CAMERAS = {}
RESOLUTION = None
for line in TRACE.open():
    frame = json.loads(line)
    if frame['type']=='metadata':
        RESOLUTION = frame['parameters']['resolution']
    if frame['type']=='fusion':
        CAMERAS[int(frame['image_stamp_ns'])] = np.array(frame['camera_to_map']['translation'][:2])


def geometric_range(row, previous):
    """旧／新stampのcamera XYとセル中心の水平距離[m]。欠けたTFはNaNにする。"""
    stamp = int(row['previous_stamp_ns'] if previous else row['stamp_ns'])
    if stamp not in CAMERAS:
        return np.nan
    center = RESOLUTION*(np.array([int(row['cell_x']),int(row['cell_y'])])+.5)
    return float(np.linalg.norm(center-CAMERAS[stamp]))


def any_worse(measures):
    """残差と実測深度距離はORに含めず、独立に扱える品質proxyだけをまとめる。"""
    # 実測深度の距離は外れ値自体で伸びるため、主判定の遠距離化はTFのXY距離を使う。
    return any(value for key,value in measures.items() if key!='farther')


def flags(row):
    """旧新品質を比較する診断flag。更新拒否の判定規則ではない。"""
    def delta(name):return float(row['new_'+name])-float(row['old_'+name])
    cue = row['cue']
    pixels = 'cell_pixels' if cue == 'obstacle_height' else 'patch_pixels'
    old_pixels, new_pixels = float(row['old_'+pixels]), float(row['new_'+pixels])
    terrain = cue != 'obstacle_height'
    return dict(farther=delta('range_m') > .05,
        geometric_farther=geometric_range(row, False)-geometric_range(row, True) > .05,
        fewer_pixels=old_pixels > 0 and new_pixels <= .8*old_pixels,
        fewer_support=terrain and delta('support_cells') <= -1,
        worse_geometry=terrain and delta('geometry_ratio') < -.1,
        more_one_sided=terrain and delta('centroid_offset_cells') > .25,
        less_side_support=cue == 'step_height' and
            min(float(row['new_forward_support']),float(row['new_rear_support'])) <
            min(float(row['old_forward_support']),float(row['old_rear_support'])))


def stratum(row):
    """同じ場所・cueと旧品質のbinで対照を揃える。新しい指標値では層別しない。"""
    old_range = geometric_range(row, True)
    support = float(row['old_support_cells'])
    pixels = float(row['old_cell_pixels' if row['cue']=='obstacle_height' else 'old_patch_pixels'])
    return (row['cell_x'],row['cell_y'],row['cue'],int(old_range/.5),
            int(support) if row['cue']!='obstacle_height' else -1, int(np.log2(max(pixels,1))))


def rates(rows):
    """更新回数を分母にflag率と品質差の中央値を出す。独立試行とはみなさない。"""
    if not rows:return dict(n=0)
    measures = [flags(r) for r in rows]
    result = dict(n=len(rows), unique_cells=len({(r['cell_x'],r['cell_y']) for r in rows}))
    for k in FLAGS:
        result[k+'_percent'] = 100*sum(m[k] for m in measures)/len(rows)
    result['any_worse_percent'] = 100*sum(any_worse(m) for m in measures)/len(rows)
    for name in ['range_m','cell_pixels','patch_pixels','support_cells','geometry_ratio','centroid_offset_cells','residual_m']:
        values = np.array([float(r['new_'+name])-float(r['old_'+name]) for r in rows])
        values = values[np.isfinite(values)]
        result['delta_'+name+'_median'] = float(np.median(values)) if len(values) else None
    return result


all_results = {}
matched_rows = []
for radius in (2,5,10):
    p = ROOT/f'radius{radius}_stride4'
    summary = json.loads((p/'summary.json').read_text())
    assert summary['parameters']['resolution']==RESOLUTION
    reference = json.loads((args.reference_root/f'radius{radius}_stride4/summary.json').read_text())
    assert summary['path']==reference['path'] and summary['path_causes']==reference['path_causes']
    rows = list(csv.DictReader((p/'quality_events.csv').open()))
    passages = list(csv.DictReader((p/'passage_black_origins.csv').open()))
    # cue単体の遷移と、セル全体のhazardの遷移を分離する。更新順に元の保持を再構成し、
    # 他cueで既に黒だったセルを「新たに黒になった」caseへ混ぜない。
    cues = ('slope_deg','roughness','step_height','obstacle_height')
    limits = np.array([summary['parameters'][k] for k in
        ('hazard_slope_limit_deg','hazard_roughness_limit','hazard_step_limit','hazard_obstacle_height_limit')])
    kept = {}; episodes = {}; timelines = defaultdict(list); cell_counts = Counter()
    def is_black(values):
        return bool((np.isfinite(values)&(np.rint(np.clip(values/limits,0,1)*100)>=100)).any())
    for key, group in itertools.groupby(rows, lambda r:(r['cell_x'],r['cell_y'],r['stamp_ns'])):
        group = list(group); cell = key[:2]
        old = kept.get(cell,np.full(4,np.nan)).copy(); new = old.copy()
        for row in group:
            i = cues.index(row['cue']); value = float(row['old_value'])
            assert np.isnan(value) and np.isnan(old[i]) or value==old[i]
            new[i] = float(row['new_value'])
        old_state = 'unknown' if not np.isfinite(old).any() else ('black' if is_black(old) else 'safe')
        new_state = 'black' if is_black(new) else 'safe'
        kind = old_state+'_to_'+new_state; cell_counts[kind]+=1
        for row in group:row['cell_transition']=kind
        if new_state=='black' and old_state!='black':
            episodes[cell]=(kind,int(key[2]))
        elif new_state!='black':
            episodes.pop(cell,None)
        kept[cell]=new
        timelines[cell].append((int(key[2]),episodes.get(cell)))
    hazard_origins = Counter(); origin_footprints = defaultdict(set); passage_cells=set()
    for row in passages:
        key=(row['cell_x'],row['cell_y'],row['passage_stamp_ns'])
        if key in passage_cells:continue
        passage_cells.add(key)
        timeline=timelines[key[:2]]
        i=bisect.bisect_left([x[0] for x in timeline],int(key[2]))-1
        assert i>=0 and timeline[i][1] is not None
        kind,stamp=timeline[i][1];hazard_origins[kind]+=1
        origin_footprints[kind].add(row['path_index'])
    starts = {(r['cell_x'],r['cell_y'],r['cue'],r['stamp_ns']):r['previous_stamp_ns'] for r in rows
              if r['transition'] in ('safe_to_black','unknown_to_black')}
    for r in passages:
        r['stamp_ns'] = r['episode_stamp_ns']
        r['previous_stamp_ns'] = starts[(r['cell_x'],r['cell_y'],r['cue'],r['episode_stamp_ns'])]
    assert not any(r['episode_transition']=='missing' for r in passages)
    result = dict(transitions={}, origin_counts=dict(Counter(r['episode_transition'] for r in passages)),
        cell_hazard_transitions=dict(cell_counts), passage_cell_hazard_origins=dict(hazard_origins),
        passage_cell_origin_footprints={k:len(v) for k,v in origin_footprints.items()})
    route_rows = [r for r in csv.DictReader((p/'path_footprints.csv').open()) if r['event']=='before_passage']
    assert len(passage_cells)==sum(int(r['black_cells']) for r in route_rows)
    clearance = [r for r in rows if r['cue']=='obstacle_height'
                 and float(r['new_value'])==0 and float(r['old_value'])>0]
    assert all(float(r['new_cell_pixels'])>=2 for r in clearance)
    result['obstacle_clearance'] = rates(clearance)
    result['obstacle_clearance']['previously_black'] = sum(r['transition']=='black_to_safe' for r in clearance)
    for cue in ('slope_deg','roughness','step_height','obstacle_height'):
        cue_rows = [r for r in rows if r['cue']==cue]
        all_groups = {kind:[r for r in cue_rows if r['transition']==kind] for kind in
                  ('safe_to_black','safe_to_safe','black_to_safe','black_to_black')}
        groups={kind:([r for r in v if r['cell_transition']==kind]
                      if kind in ('safe_to_black','safe_to_safe') else v)
                for kind,v in all_groups.items()}
        controls = defaultdict(list)
        for r in groups['safe_to_safe']:
            controls[stratum(r)].append(r)
        cases = groups['safe_to_black']; matched = [r for r in cases if stratum(r) in controls]
        matching = dict(case_total=len(cases), matched_cases=len(matched),
                        matched_unique_cells=len({(r['cell_x'],r['cell_y']) for r in matched}))
        for flag in FLAGS+['any_worse']:
            def positive(r):
                f=flags(r)
                return any_worse(f) if flag=='any_worse' else f[flag]
            matching[flag+'_case_percent'] = 100*sum(positive(r) for r in matched)/len(matched) if matched else None
            # caseが属する層の通常更新率をcase1件につき1重みで平均する。
            matching[flag+'_control_percent'] = 100*sum(
                sum(positive(c) for c in controls[stratum(r)])/len(controls[stratum(r)])
                for r in matched)/len(matched) if matched else None
        for row in matched:
            controls_here = controls[stratum(row)]
            matched_rows.append(dict(radius_m=radius, **row,
                control_count=len(controls_here),
                **{'flag_'+k:v for k,v in flags(row).items()},
                **{'control_'+k+'_percent':100*sum(flags(c)[k] for c in controls_here)/len(controls_here)
                   for k in FLAGS}))
        black_rows = [r for r in passages if r['cue']==cue]
        initial = [r for r in black_rows if r['episode_transition']=='unknown_to_black']
        overwrite = [r for r in black_rows if r['episode_transition']=='safe_to_black']
        weak = [r for r in overwrite if any_worse(flags(r))]
        result['transitions'][cue] = dict(raw={k:rates(v) for k,v in groups.items()},
            all_cue_transitions={k:rates(v) for k,v in all_groups.items()}, matched=matching,
            passage=dict(black_cue_cells=len(black_rows), initial=len(initial), overwrite=len(overwrite),
                weak_overwrite=len(weak),
                black_footprints=len({r['path_index'] for r in black_rows}),
                initial_footprints=len({r['path_index'] for r in initial}),
                overwrite_footprints=len({r['path_index'] for r in overwrite}),
                weak_overwrite_footprints=len({r['path_index'] for r in weak})))
    sets = {name:{int(r['path_index']) for r in passages if test(r)} for name,test in {
        'black':lambda r:True,
        'initial':lambda r:r['episode_transition']=='unknown_to_black',
        'overwrite':lambda r:r['episode_transition']=='safe_to_black',
        'weak_overwrite':lambda r:r['episode_transition']=='safe_to_black' and any_worse(flags(r))}.items()}
    weak_keys = {(r['path_index'],r['cell_x'],r['cell_y'],r['cue']) for r in passages
                 if r['episode_transition']=='safe_to_black' and any_worse(flags(r))}
    remaining = {int(r['path_index']) for r in passages
                 if (r['path_index'],r['cell_x'],r['cell_y'],r['cue']) not in weak_keys}
    result['footprints'] = {k:len(v) for k,v in sets.items()}
    result['footprints']['only_weak_episode_black'] = len(sets['black']-remaining)
    assert len(sets['black'])==summary['path']['before_passage']['black_footprints']
    all_results[str(radius)] = result
    print('radius',radius,'transitions',len(rows),'origins',result['origin_counts'],'footprints',result['footprints'])
    for cue, r in result['transitions'].items():
        print(cue,'matched',r['matched']['matched_cases'],
              'any worse case/control',r['matched']['any_worse_case_percent'],r['matched']['any_worse_control_percent'])
(ROOT/'quality_analysis.json').write_text(json.dumps(all_results,ensure_ascii=False,indent=2)+'\n')
with (ROOT/'matched_cases.csv').open('w', newline='') as stream:
    writer=csv.DictWriter(stream,fieldnames=list(dict.fromkeys(k for r in matched_rows for k in r)))
    writer.writeheader(); writer.writerows(matched_rows)
