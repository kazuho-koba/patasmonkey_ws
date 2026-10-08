#!/usr/bin/env python3
"""同じ採用depth/TFで、保持対照とN=1/3/5の障害物解除を一括比較する。

通常perceptionは変更しない。terrain三指標は同frameの組で保持し、
obstacleだけを再観測窓で更新する。unknownを高さ0に置換しない。
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
from pm_evaluation.height_candidates import CandidateOptions, extract_candidates, plane_reasons
from pm_evaluation.height_candidate_tracks import CandidateTracks
from pm_evaluation.obstacle_height_window import ObstacleHeightWindow
from pm_perception.mapper_replay_trace import load_mapper_trace


def residual_maxima(points, origin, shape, resolution, planes, usable):
    """保存候補枠の上限とは無関係に、全投影点の最大平面残差を集約する。"""
    h, w = shape
    index = np.floor((points[:, :2]-origin)/resolution).astype(int)
    inside = (index[:, 0] >= 0) & (index[:, 0] < w) & (index[:, 1] >= 0) & (index[:, 1] < h)
    index, selected = index[inside], points[inside]
    slots = index[:, 1]*w+index[:, 0]
    coefficients = planes.reshape(-1, 3)[slots]
    centers = origin+(index+.5)*resolution
    residual = selected[:, 2] - (coefficients[:, :2]*(selected[:, :2]-centers)).sum(axis=1)-coefficients[:, 2]
    result = np.full(h*w, -np.inf)
    np.maximum.at(result, slots, residual)
    result[(~usable.reshape(-1)) | (~np.isfinite(result))] = np.nan
    return np.maximum(result, 0.)


def write_csv(path, rows):
    """CSVを明示的に閉じ、空集合もヘッダなしの空ファイルとして残す。"""
    with path.open('w', newline='') as stream:
        if rows:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag'); parser.add_argument('--mapper-trace', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--config', default=str(Path(__file__).resolve().parents[1]/'config/height_candidate_diagnosis.yaml'))
    args = parser.parse_args()
    meta, frames, _ = load_mapper_trace(args.mapper_trace)
    params = meta['parameters']; resolution = params['resolution']
    options = CandidateOptions(**yaml.safe_load(Path(args.config).read_text()))
    limits = np.array([params['hazard_slope_limit_deg'], params['hazard_roughness_limit'],
                       params['hazard_step_limit'], params['hazard_obstacle_height_limit']])
    output = Path(args.output); output.mkdir(parents=True, exist_ok=False)
    # 対応探索は一度だけ行い、trackの直近5観測から各Nの支持を読む。
    tracker = CandidateTracks(5, .05, .03)
    track_history = defaultdict(lambda: deque(maxlen=5))
    stores = {n: {} for n in (1, 3, 5)}
    latched, terrain = {}, {}
    path = []
    for stamp, frame in frames.items():
        base = frame['base_to_map']
        pose = pose_for_footprint(stamp, base['translation']+base['quaternion'])
        if not path or np.hypot(pose[1]-path[-1][1], pose[2]-path[-1][2]) >= .25:
            path.append(pose)
    rows, black_rows, events, seen, timings = [], [], [], [], []
    counts = Counter(); cursor = 0
    began, cpu = time.perf_counter(), time.process_time()

    def emit(pose):
        cells = footprint_cells(pose, resolution, .45, .55)
        for mode in ('latched', 'N1', 'N3', 'N5'):
            store = None if mode == 'latched' else stores[int(mode[1:])]
            values = [list(terrain.get(cell, [np.nan]*3))+[
                latched.get(cell, np.nan) if store is None else
                store[cell].value if cell in store else np.nan] for cell in cells]
            rows.append(dict(stamp_ns=pose[0], mode=mode, **classify(values, limits)))
            for cell, value in zip(cells, values):
                flags = np.isfinite(value) & (np.rint(np.clip(np.asarray(value)/limits, 0, 1)*100) >= 100)
                if flags.any():
                    state = None if store is None else store.get(cell)
                    black_rows.append(dict(stamp_ns=pose[0], mode=mode, cell_x=cell[0], cell_y=cell[1],
                        causes='+'.join(name for name, flag in zip(('slope','roughness','step','obstacle'), flags) if flag),
                        slope_deg=value[0], roughness_m=value[1], step_m=value[2], obstacle_m=value[3],
                        obstacle_state=('confirmed' if state.confirmed else 'pending') if state and state.active else 'inactive'))

    for _, _, record in messages(args.bag, [params['depth_topic']]):
        depth = deserialize_message(record.data, Image); stamp = stamp_ns(depth)
        if stamp not in frames:
            continue
        # 未来の情報が過去の通過評価へ漏れないよう、先に通過ラベルを確定する。
        while cursor < len(path) and path[cursor][0] <= stamp:
            emit(path[cursor]); cursor += 1
        tick = time.perf_counter()
        points, _, u, v, pose, baseline, planes = prepare_frame(depth, frames[stamp], params)
        reasons = plane_reasons(baseline, params)
        candidates, layers = extract_candidates(points, np.column_stack((u, v)), baseline['origin'],
            baseline['hazard'].shape, resolution, planes, reasons['plane_usable'], limits[3], options)
        heights = residual_maxima(points, baseline['origin'], baseline['hazard'].shape,
                                  resolution, planes, reasons['plane_usable'])
        origin = np.rint(baseline['origin']/resolution).astype(int)
        width = baseline['hazard'].shape[1]; groups = defaultdict(list)
        for candidate in candidates:
            slot = candidate['slot']; cell = (int(origin[0]+slot % width), int(origin[1]+slot//width))
            candidate['high_hit'] = bool(candidate['max_residual_m'] >= limits[3]) if candidate['plane_usable'] else None
            groups[cell].append(candidate)
        for slot in np.flatnonzero(layers['point_count']):
            cell = (int(origin[0]+slot % width), int(origin[1]+slot//width))
            yy, xx = divmod(int(slot), width)
            cues = np.array([baseline[name][yy, xx] for name in ('slope_deg','roughness','step_height')])
            if np.isfinite(cues).all():
                terrain[cell] = cues.tolist()
            hit = layers['above_plane_hit'][yy, xx]
            if np.isfinite(hit):
                latched[cell] = max(latched.get(cell, 0.), hit*limits[3])
            height = float(heights[slot])
            # 全点集約と既存hitが一致することを各frameで確認する。
            if np.isfinite(hit):
                assert bool(height >= limits[3]) == bool(hit)
            tracking = tracker.update(cell, groups[cell], stamp, len(seen))
            histories = []
            for candidate, tracked in zip(groups[cell], tracking):
                history = track_history[tracked['track_id']]
                history.append((stamp, candidate['high_hit']))
                histories.append(list(history))
            if not np.isfinite(height):
                counts['plane_unknown_cell_observations'] += 1
                continue
            counts['valid_cell_observations'] += 1
            for n, store in stores.items():
                state = store.setdefault(cell, ObstacleHeightWindow(n, limits[3]))
                # 今回の黒episodeより前の支持を再利用しない。未対応frameは中立だが、
                # 対応群のplane unknownは「有効な連続支持」の条件を満たさない。
                onset = state.onset if state.active else stamp
                confirmed = any(len(h) >= n and all(s >= onset and value is True for s, value in h[-n:]) for h in histories)
                was_active = state.active
                previous = state.update(height, stamp, confirmed)
                if height >= limits[3] and not was_active:
                    counts['N%d_new_black' % n] += 1
                if previous:
                    counts['N%d_clear_%s' % (n, previous)] += 1
                    events.append(dict(window=n, cell_x=cell[0], cell_y=cell[1], stamp_ns=stamp,
                        previous_state=previous, onset_stamp_ns=state.onset,
                        seconds_since_onset=(stamp-state.onset)/1e9,
                        current_height_m=height, mean_height_m=state.value,
                        observations=len(state.history), point_count=int(layers['point_count'][yy, xx])))
        seen.append(stamp); timings.append((time.perf_counter()-tick)*1000)
        if len(seen) % 200 == 0:
            print('evaluated %d frames' % len(seen), flush=True)
    if seen != list(frames):
        raise ValueError('採用画像列がtraceと一致しません')
    while cursor < len(path):
        emit(path[cursor]); cursor += 1
    results = {}
    for mode in ('latched','N1','N3','N5'):
        subset = [r for r in rows if r['mode'] == mode]
        sums = {key: sum(r[key] for r in subset) for key in ('cells','known_cells','black_cells','all_cues_known_cells')}
        names = ('slope','roughness','step','obstacle')
        results[mode] = dict(**sums, footprints=len(subset),
            black_footprints=sum(r['black_cells'] > 0 for r in subset),
            black_percent_all_footprints=100*sum(r['black_cells'] > 0 for r in subset)/len(subset),
            unknown_cell_percent=100*(1-sums['known_cells']/sums['cells']),
            fully_known_footprints=sum(r['all_cues_known_cells'] == r['cells'] for r in subset),
            overlapping_causes={name: sum(r[name+'_black_cells'] > 0 for r in subset) for name in names},
            exclusive_causes=dict(Counter('+'.join(name for name in names if r[name+'_black_cells']) for r in subset if r['black_cells'])))
    summary = dict(parameters=params, candidate_options=vars(options), adopted_frames=len(seen),
        results=results, counts=dict(counts), trace_sha256=hashlib.sha256(Path(args.mapper_trace).read_bytes()).hexdigest(),
        frame_ms=dict(mean=float(np.mean(timings)), p95=float(np.percentile(timings,95)), max=max(timings)),
        elapsed_seconds=time.perf_counter()-began, process_cpu_seconds=time.process_time()-cpu,
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
        limitations=['低い最大残差はfree-ray証拠ではない', 'terrain三指標は変更しない',
                     '同一セルの異なる表面・視点も窓に含むため真の障害物の誤解除は未検証',
                     '未知の平面・未観測は窓に入れず保持する', 'オフラインtrack一覧は無制限'])
    summary['source_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (Path(__file__).resolve(), Path(__file__).resolve().parents[1]/'pm_evaluation/obstacle_height_window.py')}
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    write_csv(output/'footprints.csv', rows); write_csv(output/'black_cells.csv', black_rows)
    write_csv(output/'clear_events.csv', events)
    print(json.dumps(results, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
