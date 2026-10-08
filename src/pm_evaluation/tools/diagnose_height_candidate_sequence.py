#!/usr/bin/env python3
"""連続採用frameで候補対応とN=1／3を比較する（通常nodeは変更しない）。

確認済みだけの出力は診断用で、安全出力ではない。保守側出力には初回high-hitも
保持する。free-ray証拠をまだ検証していないため、危険候補の解除は行わない。
姿勢補正や高さのセル間融合はなく、terrain三指標は同frameの組を保持する。
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
from evaluate_single_depth_terrain import messages, stamp_ns
from evaluate_independent_depth_frames import pose_for_footprint
from evaluate_frame_feature_coverage import footprint_cells
from pm_evaluation.height_candidates import CandidateOptions, extract_candidates, plane_reasons
from pm_evaluation.height_candidate_tracks import CandidateTracks
from pm_perception.mapper_replay_trace import load_mapper_trace


def classify(values, limits):
    """unknownを分母から隠さず、全footprintと既知footprintの両方で黒率を出す。"""
    values = np.asarray(values)
    known = np.isfinite(values).any(axis=1)
    by_cue = np.isfinite(values) & (np.rint(np.clip(values/limits, 0, 1)*100) >= 100)
    black = by_cue.any(axis=1)
    return dict(cells=len(values), known_cells=int(known.sum()), black_cells=int(black.sum()),
                all_cues_known_cells=int(np.isfinite(values).all(axis=1).sum()),
                **{name+'_black_cells': int(by_cue[:, i].sum()) for i, name in enumerate(
                    ('slope', 'roughness', 'step', 'obstacle'))})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag'); parser.add_argument('--mapper-trace', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--config', default=str(Path(__file__).resolve().parents[1]/'config/height_candidate_diagnosis.yaml'))
    parser.add_argument('--z-match-gate', type=float, default=.05)
    parser.add_argument('--xy-match-gate', type=float, default=.03)
    parser.add_argument('--window', type=int, default=3,
                        help='対応観測の短期窓N。既定3、N=5比較では5を指定')
    args = parser.parse_args()
    if args.window < 1:
        parser.error('windowは1以上です')
    confirmed_mode = 'N%d_confirmed' % args.window
    conservative_mode = 'N%d_conservative' % args.window
    options = CandidateOptions(**yaml.safe_load(Path(args.config).read_text()))
    meta, frames, _ = load_mapper_trace(args.mapper_trace)
    params = dict(meta['parameters']); res = params['resolution']
    output = Path(args.output); output.mkdir(parents=True, exist_ok=False)
    # N=1の値は現在候補から直接得られるので、同じtrackをもう一式保持しない。
    model = CandidateTracks(args.window, args.z_match_gate, args.xy_match_gate)
    # 時間ではなく0.25m移動ごとに通過footprintを選び、同じ位置の長い停止を過重にしない。
    path = []
    for stamp, frame in frames.items():
        base = frame['base_to_map']
        pose = pose_for_footprint(stamp, base['translation']+base['quaternion'])
        if not path or np.hypot(pose[1]-path[-1][1], pose[2]-path[-1][2]) >= .25:
            path.append(pose)
    limits = np.array([params['hazard_slope_limit_deg'], params['hazard_roughness_limit'],
                       params['hazard_step_limit'], params['hazard_obstacle_height_limit']])
    # 最後の有効なterrain三指標を一組で保持。obstacleと別frameで得た値を混ぜて
    # slope等を再fitしない。距離品質更新は今回は入れずN比較以外の条件を一致させる。
    terrain, obstacles = {}, {k: {} for k in ('raw_width', 'N1', confirmed_mode, conservative_mode)}
    rows, frame_rows, seen, cursor = [], [], [], 0
    totals = Counter(); deltas = []; averaged_deltas = []; previous_mean = {}
    kind_deltas = defaultdict(list); kind_mean_deltas = defaultdict(list)
    first_hit, first_confirm = {}, {}
    terrain_stamps, black_cell_rows = {}, []
    observation_state = {}
    latest_depth_stamp = None
    recent_frames = deque(maxlen=args.window)
    pending_rows = []
    cell_observation_counts = Counter()
    first_hit_observation_counts = {}
    track_confirm_delays, track_confirm_motion = [], []
    timing = []; began = time.perf_counter(); cpu = time.process_time()

    def emit(pose):
        cells = footprint_cells(pose, res, .45, .55)
        for mode, store in obstacles.items():
            values = [list(terrain.get(cell, [np.nan]*3))+[store.get(cell, np.nan)] for cell in cells]
            rows.append(dict(stamp_ns=pose[0], mode=mode, **classify(values, limits)))
            # 黒の根拠値と出典stampだけ保存。map／全画素を再度複製しない。
            for cell, value in zip(cells, values):
                flags = np.isfinite(value) & (np.rint(np.clip(np.asarray(value)/limits, 0, 1)*100) >= 100)
                if not flags.any():
                    continue
                source = first_confirm if mode == confirmed_mode else first_hit
                black_cell_rows.append(dict(passage_stamp_ns=pose[0], mode=mode,
                    cell_x=cell[0], cell_y=cell[1],
                    causes='+'.join(name for name, flag in zip(('slope','roughness','step','obstacle'), flags) if flag),
                    slope_deg=value[0], roughness_m=value[1], step_m=value[2], obstacle_m=value[3],
                    terrain_stamp_ns=terrain_stamps.get(cell),
                    obstacle_first_positive_stamp_ns=source.get(cell, [None])[0],
                    latest_depth_stamp_ns=latest_depth_stamp,
                    last_cell_observation_stamp_ns=observation_state.get(cell, {}).get('stamp'),
                    **{key: observation_state.get(cell, {}).get(key) for key in
                       ('plane_reason_bits', 'support_count', 'above_plane_hit',
                       'current_window_confirmed')}))
                if (mode == conservative_mode and flags[3]
                        and obstacles[confirmed_mode].get(cell) != limits[3]):
                    # 確認待ちの最も支持が多いhigh trackを診断する。実行時の選択規則ではない。
                    # 全5画像窓は別に数え、未観測を対応候補のhistoryへ埋めない。
                    tracks = [t for t in model.cells.get(cell, []) if 'first_positive_stamp' in t]
                    best = max(tracks, key=lambda t: (sum(h['high_hit'] is True for h in t['history']),
                                                      len(t['history']), t['stamp'])) if tracks else None
                    history = list(best['history']) if best else []
                    cell_observations = [f[cell] for f in recent_frames if cell in f]
                    pending_rows.append(dict(passage_stamp_ns=pose[0], cell_x=cell[0], cell_y=cell[1],
                        positive_track_count=len(tracks), best_track_id=best['id'] if best else None,
                        matched_window_observations=len(history),
                        matched_window_positive=sum(h['high_hit'] is True for h in history),
                        matched_window_valid_no_hit=sum(h['high_hit'] is False for h in history),
                        matched_window_plane_unknown=sum(h['high_hit'] is None for h in history),
                        recent_image_count=len(recent_frames), recent_cell_observed=len(cell_observations),
                        recent_plane_valid=sum(o[0] == 0 for o in cell_observations),
                        recent_cell_high_hit=sum(o[1] == 1 for o in cell_observations),
                        total_cell_observations=cell_observation_counts[cell],
                        cell_observations_from_first_hit=cell_observation_counts[cell]
                        -first_hit_observation_counts[cell]+1))

    with (output/'tracks.csv').open('w', newline='') as stream:
        writer = None
        for _, _, record in messages(args.bag, [params['depth_topic']]):
            depth = deserialize_message(record.data, Image); stamp = stamp_ns(depth)
            if stamp not in frames:
                continue
            # 同時刻・未来frameを通過前の証拠として使わない。
            while cursor < len(path) and path[cursor][0] <= stamp:
                emit(path[cursor]); cursor += 1
            tick = time.perf_counter()
            points, optical, u, v, pose, baseline, planes = prepare_frame(depth, frames[stamp], params)
            reasons = plane_reasons(baseline, params)
            candidates, layers = extract_candidates(points, np.column_stack((u, v)), baseline['origin'],
                baseline['hazard'].shape, res, planes, reasons['plane_usable'], limits[3], options)
            origin = np.rint(baseline['origin']/res).astype(int); width = baseline['hazard'].shape[1]
            groups = defaultdict(list)
            for candidate in candidates:
                slot = candidate['slot']; cell = (int(origin[0]+slot % width), int(origin[1]+slot//width))
                candidate['high_hit'] = (bool(candidate['max_residual_m'] >= limits[3])
                                         if candidate['plane_usable'] else None)
                groups[cell].append(candidate)
            fstats = Counter()
            frame_observations = {}
            for slot in np.flatnonzero(layers['point_count']):
                cell = (int(origin[0]+slot % width), int(origin[1]+slot//width))
                cell_observation_counts[cell] += 1
                yy, xx = divmod(int(slot), width)
                cues = np.array([baseline[name][yy, xx] for name in ('slope_deg', 'roughness', 'step_height')])
                if np.isfinite(cues).all():
                    terrain[cell] = cues.tolist()
                    terrain_stamps[cell] = stamp
                raw = float(layers['raw_span'][yy, xx]); hit = layers['above_plane_hit'][yy, xx]
                # 対照も負のfree証拠がない限りblackを保持。通常Aのclear規則とは別対照。
                for mode, value in [('raw_width', raw), ('N1', hit*limits[3]), (conservative_mode, hit*limits[3])]:
                    if np.isfinite(value):
                        obstacles[mode][cell] = max(value, obstacles[mode].get(cell, 0.))
                if hit == 1:
                    first_hit.setdefault(cell, (stamp, pose[1], pose[2]))
                    first_hit_observation_counts.setdefault(cell, cell_observation_counts[cell])
                bits = int(reasons['plane_reason_bits'][yy, xx])
                if raw/limits[3] >= .995 and not np.isfinite(hit):
                    fstats['raw_black_plane_unknown'] += 1
                    for bit, name in [(1,'self'), (2,'support'), (4,'degenerate'), (8,'slope'), (16,'roughness')]:
                        fstats['unknown_'+name] += bool(bits & bit)
                fstats['observed_cells'] += 1
                fstats['overflow_cells'] += int(layers['overflow_count'][yy, xx] > 0)
                fstats['overflow_groups'] += int(layers['overflow_count'][yy, xx])
                fstats['high_hit_cells'] += int(hit == 1)
                n3 = model.update(cell, groups[cell], stamp, len(seen))
                for candidate, tracking in zip(groups[cell], n3):
                    row = dict(stamp_ns=stamp, cell_x=cell[0], cell_y=cell[1], kind=candidate['kind'],
                               count=candidate['count'], z_mean_m=candidate['z_mean_m'],
                               high_hit=candidate['high_hit'], **tracking)
                    if writer is None:
                        writer = csv.DictWriter(stream, fieldnames=list(row)); writer.writeheader()
                    writer.writerow(row)
                    if tracking['delta_z_m'] is not None:
                        deltas.append(abs(tracking['delta_z_m']))
                        averaged_deltas.append(abs(tracking['mean_z_m']-previous_mean[tracking['track_id']]))
                        kind_deltas[candidate['kind']].append(deltas[-1])
                        kind_mean_deltas[candidate['kind']].append(averaged_deltas[-1])
                    previous_mean[tracking['track_id']] = tracking['mean_z_m']
                    if tracking['first_confirmation']:
                        first_stamp = tracking['first_positive_stamp']
                        track_confirm_delays.append((stamp-first_stamp)/1e9)
                        old_xy = frames[first_stamp]['base_to_map']['translation'][:2]
                        track_confirm_motion.append(float(np.linalg.norm(np.array(pose[1:3])-old_xy)))
                confirmed = any(r['confirmed'] for r in n3)
                # 「未知だから黒」と「未知の再観測で過去の黒が残る」を区別する根拠。
                # ここは診断のみで、既存の更新・保持規則を変えない。
                observation_state[cell] = dict(stamp=stamp, plane_reason_bits=bits,
                    support_count=int(baseline['support_count'][yy, xx]),
                    above_plane_hit=int(hit) if np.isfinite(hit) else None,
                    current_window_confirmed=confirmed)
                frame_observations[cell] = (bits, int(hit) if np.isfinite(hit) else None)
                if confirmed:
                    obstacles[confirmed_mode][cell] = limits[3]
                    first_confirm.setdefault(cell, (stamp, pose[1], pose[2]))
                elif hit == 0 and cell not in obstacles[confirmed_mode]:
                    obstacles[confirmed_mode][cell] = 0.
                # overflowの直接hitは保守側に残るが、対応不能なので確認済みにはしない。
                fstats['confirmed_cells'] += int(confirmed)
                fstats['pending_hit_cells'] += int(hit == 1 and not confirmed)
            totals.update(fstats)
            timing.append((time.perf_counter()-tick)*1000)
            frame_rows.append(dict(stamp_ns=stamp, **fstats))
            seen.append(stamp)
            latest_depth_stamp = stamp
            recent_frames.append(frame_observations)
            if len(seen) % 200 == 0:
                print('evaluated %d frames' % len(seen), flush=True)
    if seen != list(frames):
        raise ValueError('traceと採用画像列が一致しません')
    while cursor < len(path):
        emit(path[cursor]); cursor += 1

    def distribution(values):
        return dict(count=len(values), mean=float(np.mean(values)), p95=float(np.percentile(values,95)),
                    max=float(max(values))) if values else dict(count=0)

    results = {}
    for mode in obstacles:
        subset = [r for r in rows if r['mode'] == mode]
        sums = {key: sum(r[key] for r in subset) for key in ('cells','known_cells','black_cells','all_cues_known_cells')}
        black = sum(r['black_cells'] > 0 for r in subset); known = sum(r['known_cells'] > 0 for r in subset)
        results[mode] = dict(**sums, footprints=len(subset), black_footprints=black,
            black_percent_all_footprints=black/len(subset)*100,
            black_percent_known_footprints=black/known*100 if known else None,
            all_unknown_footprints=sum(r['known_cells'] == 0 for r in subset),
            unknown_cell_percent=(1-sums['known_cells']/sums['cells'])*100,
            fully_known_footprints=sum(r['all_cues_known_cells'] == r['cells'] for r in subset))
        names = ('slope', 'roughness', 'step', 'obstacle')
        # footprint内の異なるセルに異なる原因がある場合も重複として保持する。
        # 排他的組合せの和はblack_footprintsと一致する。
        combinations = Counter('+'.join(name for name in names if r[name+'_black_cells'] > 0)
                               for r in subset if r['black_cells'] > 0)
        results[mode]['cause_analysis'] = dict(
            overlapping_footprints={name: sum(r[name+'_black_cells'] > 0 for r in subset) for name in names},
            overlapping_black_cells={name: sum(r[name+'_black_cells'] for r in subset) for name in names},
            exclusive_footprint_combinations=dict(combinations))
    delays = [(first_confirm[cell][0]-old[0])/1e9 for cell, old in first_hit.items() if cell in first_confirm]
    distances = [np.hypot(first_confirm[cell][1]-old[1], first_confirm[cell][2]-old[2])
                 for cell, old in first_hit.items() if cell in first_confirm]
    summary = dict(parameters=params, candidate_options=vars(options), adopted_frames=len(seen),
        window_observations=args.window,
        matching=dict(z_gate_m=args.z_match_gate, xy_gate_m=args.xy_match_gate, **model.stats),
        totals=dict(totals), matched_height_change_m=distribution(deltas),
        matched_short_mean_change_m=distribution(averaged_deltas),
        matched_height_change_by_kind_m={k: distribution(v) for k,v in kind_deltas.items()},
        matched_short_mean_change_by_kind_m={k: distribution(v) for k,v in kind_mean_deltas.items()},
        cell_first_hit_to_first_confirmation_seconds=distribution(delays),
        cell_first_hit_to_first_confirmation_motion_m=distribution(distances),
        track_confirmation_delay_seconds=distribution(track_confirm_delays),
        track_confirmation_motion_m=distribution(track_confirm_motion),
        hit_cells_ever=len(first_hit), confirmed_cells_ever=len(first_confirm), results=results,
        frame_ms=distribution(timing), elapsed_seconds=time.perf_counter()-began,
        process_cpu_seconds=time.process_time()-cpu,
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
        matching_tracks=len(previous_mean),
        trace_sha256=hashlib.sha256(Path(args.mapper_trace).read_bytes()).hexdigest(),
        limitations=[confirmed_mode+' is not a safe output; pending hazards are excluded only for diagnosis',
            'No free-space evidence or obstacle clearing; missing/occluded is neutral',
            'No calibrated existence probability; count is positive support only',
            'No pose correction; interval matching can merge touching height groups',
            'No terrain recomputation from averaged heights; N comparison affects obstacle evidence only',
            'Footprint .45 x .55m proxy; current-frame three terrain cues held together without quality gating',
            'Bounded observations per track, but track inventory is offline/unbounded'])
    summary['source_sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (Path(__file__).resolve(), Path(__file__).resolve().parents[1]/'pm_evaluation/height_candidate_tracks.py',
                     Path(__file__).resolve().parents[1]/'pm_evaluation/height_candidates.py')}
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    (output/'frames.json').write_text(json.dumps(frame_rows, ensure_ascii=False, indent=2)+'\n')
    with (output/'footprints.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    with (output/'black_cells.csv').open('w', newline='') as stream:
        if black_cell_rows:
            writer = csv.DictWriter(stream, fieldnames=list(black_cell_rows[0]))
            writer.writeheader(); writer.writerows(black_cell_rows)
    with (output/'pending_obstacle_cells.csv').open('w', newline='') as stream:
        if pending_rows:
            writer = csv.DictWriter(stream, fieldnames=list(pending_rows[0]))
            writer.writeheader(); writer.writerows(pending_rows)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
