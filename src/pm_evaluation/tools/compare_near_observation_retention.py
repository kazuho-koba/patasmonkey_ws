#!/usr/bin/env python3
"""保存済み独立frame更新を再適用し、点数条件付き近距離優先を比較する。

入力は品質診断の全有効更新。高さ融合や指標の再計算はしないため、採用TF・
画素・指標はA/Bで完全に共通。経路セルの最終通過までの記録だけで通過評価は
再現できるが、全地図coverageや動的障害物の検出性能を評価するものではない。
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from evaluate_spatial_feature_coverage import describe, aggregate, cause_summary, footprint_cells
from evaluate_independent_depth_frames import pose_for_footprint
from pm_perception.mapper_replay_trace import load_mapper_trace


def reject_update(old_distance, old_points, new_distance, new_points, margin=.05):
    """距離[m]が許容幅を超えて増し、旧点数が新点数以上なら拒否する。

    点数不足の旧観測には優先権を与えない。NaN品質も拒否根拠にせず更新を許可。
    黒への更新だけでなく解除も同条件で扱い、その副作用を別集計する。
    """
    return (all(np.isfinite(x) for x in (old_distance, old_points, new_distance, new_points))
            and new_distance > old_distance+margin and old_points >= new_points)


def run(source, trace, margin):
    """A/Bを並行再生し、通過stampと同時の更新は通過評価後に適用する。"""
    summary = json.loads((source/'summary.json').read_text())
    _, frames, _ = load_mapper_trace(trace)
    p = summary['parameters']; resolution = p['resolution']
    limits = np.array([p[k] for k in ('hazard_slope_limit_deg', 'hazard_roughness_limit',
                                    'hazard_step_limit', 'hazard_obstacle_height_limit')])
    cues = ('slope_deg', 'roughness', 'step_height', 'obstacle_height')
    path = []
    for stamp, frame in frames.items():
        base = frame['base_to_map']
        pose = pose_for_footprint(stamp, base['translation']+base['quaternion'])
        if not path or np.hypot(pose[1]-path[-1][1], pose[2]-path[-1][2]) >= summary['path_spacing']:
            path.append(pose)
    stores = [{}, {}]; rows = [[], []]; index = 0
    counters = {cue: dict(updates=0, rejected=0, rejected_black=0,
                         rejected_clear_zero=0, rejected_black_to_nonblack=0) for cue in cues}

    def evaluate_passage():
        pose = path[index]
        cells = footprint_cells(pose, resolution, summary['footprint_width'], summary['footprint_length'])
        for mode in range(2):
            values = np.array([stores[mode].get(c, (np.full(4, np.nan), None, None))[0] for c in cells])
            rows[mode].append(dict(path_index=index, passage_stamp_ns=int(pose[0]),
                                   **describe(values, limits)))

    previous_stamp = -1
    with (source/'quality_events.csv').open() as stream:
        for event in csv.DictReader(stream):
            stamp = int(event['stamp_ns'])
            if stamp < previous_stamp:
                raise ValueError('更新stamp順序が逆転')
            previous_stamp = stamp
            while index < len(path) and int(path[index][0]) <= stamp:
                evaluate_passage(); index += 1
            cell = (int(event['cell_x']), int(event['cell_y']))
            i = cues.index(event['cue']); value = float(event['new_value'])
            camera = frames[stamp]['camera_to_map']['translation']
            distance = float(np.hypot((cell[0]+.5)*resolution-camera[0],
                                      (cell[1]+.5)*resolution-camera[1]))
            points = float(event['new_cell_pixels' if i == 3 else 'new_patch_pixels'])
            counters[cues[i]]['updates'] += 1
            for mode in range(2):
                if cell not in stores[mode]:
                    stores[mode][cell] = (np.full(4, np.nan), np.full(4, np.nan), np.full(4, np.nan))
                values, distances, counts = stores[mode][cell]
                # Bは「最後に受理した」観測と比較する。拒否された観測の品質で
                # 基準を更新してしまうと、近距離優先の保持規則にならない。
                if mode and reject_update(distances[i], counts[i], distance, points, margin):
                    stats = counters[cues[i]]; stats['rejected'] += 1
                    black_new = np.rint(np.clip(value/limits[i], 0, 1)*100) >= 100
                    black_old = np.rint(np.clip(values[i]/limits[i], 0, 1)*100) >= 100
                    stats['rejected_black'] += int(black_new)
                    stats['rejected_clear_zero'] += int(i == 3 and value == 0 and values[i] > 0)
                    stats['rejected_black_to_nonblack'] += int(black_old and not black_new)
                    continue
                values[i], distances[i], counts[i] = value, distance, points
    while index < len(path):
        evaluate_passage(); index += 1
    # 診断ログの欠落・分母の取り違えを検出。Aの全通過行を元評価と整数で照合。
    with (source/'path_footprints.csv').open() as stream:
        reference = [r for r in csv.DictReader(stream) if r['event'] == 'before_passage']
    assert len(reference) == len(rows[0])
    for got, expected in zip(rows[0], reference):
        for key, value in got.items():
            assert value == int(expected[key]), (key, value, expected[key])
    changes = []
    for a, b in zip(*rows):
        if a != b:
            changes.append(dict(path_index=a['path_index'], baseline=a, prefer_near=b))
    return dict(radius_m=summary['radius_m'], max_depth=p['max_depth'], stride=summary['stride'],
                distance_margin_m=margin, baseline_verified=True,
                modes={name: dict(path=aggregate(r), causes=cause_summary(r))
                       for name, r in zip(('baseline', 'prefer_near'), rows)},
                rejected_updates=counters, changed_passages=changes), rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--mapper-trace', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--distance-margin', type=float, default=.05)
    args = parser.parse_args()
    if not np.isfinite(args.distance_margin) or args.distance_margin < 0:
        parser.error('距離許容幅は非負の有限値')
    result, rows = run(args.source, args.mapper_trace, args.distance_margin)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'comparison.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    for name, data in zip(('baseline', 'prefer_near'), rows):
        with (args.output/(name+'_passages.csv')).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(data[0]))
            writer.writeheader(); writer.writerows(data)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
