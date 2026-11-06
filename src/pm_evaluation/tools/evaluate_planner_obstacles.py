#!/usr/bin/env python3
"""保存済み撮像時刻TFと本実装で、7月bagのobstacle解除を対照評価する。

画像受信や実時間性能の試験ではない。通過より前の画像だけでfootprintを評価し、
unknownを分母から落とさない。解除なし対照も同じrolling領域に限定する。
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from evaluate_single_depth_terrain import messages, stamp_ns
from evaluate_independent_depth_frames import recorded_transform, pose_for_footprint
from evaluate_frame_feature_coverage import footprint_cells
from pm_perception.depth_projection import sampled_points, transform_points
from pm_perception.independent_obstacles import IndependentObstacles
from pm_perception.mapper_replay_trace import load_mapper_trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag'); parser.add_argument('--mapper-trace', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output); output.mkdir(parents=True, exist_ok=False)
    meta, frames, _ = load_mapper_trace(args.mapper_trace)
    params = meta['parameters']
    model = IndependentObstacles()
    path = []
    for stamp, frame in frames.items():
        base = frame['base_to_map']
        pose = pose_for_footprint(stamp, base['translation']+base['quaternion'])
        if not path or np.hypot(pose[1]-path[-1][1], pose[2]-path[-1][2]) >= .25:
            path.append(pose)
    cursor = 0; rows = []; seen = []; clears = dict(to_free=0, to_unknown=0,
        unknown_no_current_points=0, unknown_current_points_but_invalid=0)
    latched = set(); times = []

    def emit(pose):
        cells = footprint_cells(pose, model.res, .45, .55)
        for mode in ('N3', 'no_clear'):
            values = [100 if mode == 'no_clear' and cell in latched else
                      model.cells.get(cell, {}).get('obstacle', -1) for cell in cells]
            rows.append(dict(stamp_ns=pose[0], mode=mode, cells=len(values),
                             black=sum(v == 100 for v in values),
                             unknown=sum(v < 0 for v in values)))

    for _, _, record in messages(args.bag, [params['depth_topic']]):
        depth = deserialize_message(record.data, Image); stamp = stamp_ns(depth)
        if stamp not in frames: continue
        # 同時刻の画像も通過直前評価へ混ぜず、未来情報漏洩を防ぐ。
        while cursor < len(path) and path[cursor][0] <= stamp:
            emit(path[cursor]); cursor += 1
        frame = frames[stamp]; base = frame['base_to_map']
        points = sampled_points(depth, *frame['intrinsics'], 4, .4, 5.)
        points = transform_points(points, recorded_transform(frame['camera_to_map']))
        pose = pose_for_footprint(stamp, base['translation']+base['quaternion'])
        before = {k for k, v in model.cells.items() if v['active']}
        tick = time.perf_counter()
        model.update(points, frame['camera_to_map']['translation'], base['translation'], stamp, pose[-1])
        times.append((time.perf_counter()-tick)*1000)
        # 解除なし対照は危険発見時だけ追加し、空間破棄は本方式と揃える。
        active = {k for k, v in model.cells.items() if v['active']}
        latched.intersection_update(model.cells); latched.update(active)
        observed = {tuple(x) for x in np.floor(points[:, :2]/model.res).astype(int)}
        for key in before-active:
            if key in model.cells:
                clears['to_free' if model.cells[key]['obstacle'] == 0 else 'to_unknown'] += 1
                if model.cells[key]['obstacle'] < 0:
                    clears['unknown_current_points_but_invalid' if key in observed
                           else 'unknown_no_current_points'] += 1
        seen.append(stamp)
        if len(seen) % 200 == 0: print('frames', len(seen), flush=True)
    if seen != list(frames): raise ValueError('bagとtraceの採用画像列が不一致')
    while cursor < len(path): emit(path[cursor]); cursor += 1
    results = {}
    for mode in ('N3', 'no_clear'):
        r = [row for row in rows if row['mode'] == mode]; total = sum(x['cells'] for x in r)
        results[mode] = dict(footprints=len(r), black_footprints=sum(x['black'] > 0 for x in r),
            black_footprint_percent=100*sum(x['black'] > 0 for x in r)/len(r),
            unknown_cell_percent=100*sum(x['unknown'] for x in r)/total,
            black_cell_percent=100*sum(x['black'] for x in r)/total,
            fully_known_footprints=sum(x['unknown'] == 0 for x in r),
            completely_unknown_footprints=sum(x['unknown'] == x['cells'] for x in r))
    paired = list(zip(rows[::2], rows[1::2]))
    previous_black = [a for a,b in paired if b['black'] > 0]
    transition = dict(previous_black_footprints=len(previous_black),
        now_fully_known=sum(a['unknown'] == 0 for a in previous_black),
        now_partly_unknown=sum(0 < a['unknown'] < a['cells'] for a in previous_black),
        now_fully_unknown=sum(a['unknown'] == a['cells'] for a in previous_black))
    summary = dict(bag=args.bag, trace=args.mapper_trace, frames=len(seen), results=results,
                   previous_black_transition=transition,
                   clear_events=clears, counters=model.counters,
                   settings=dict(resolution=.1, size=8., min_depth=.4, max_depth=5.,
                                 pixel_stride=4, obstacle_limit=.08, window=3),
                   trace_sha256=hashlib.sha256(Path(args.mapper_trace).read_bytes()).hexdigest(),
                   computation_ms=dict(mean=float(np.mean(times)), p95=float(np.percentile(times,95)),
                                       maximum=float(np.max(times))))
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    with (output/'footprints.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__': main()
