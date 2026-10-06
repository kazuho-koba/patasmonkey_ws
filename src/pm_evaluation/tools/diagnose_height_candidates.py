#!/usr/bin/env python3
"""実使用TFで独立depthの候補表現を診断し、同条件の従来評価と並べる。

ROS nodeを起動せずMCAPをstreamする。通常mapperの高さ融合・hazard出力は変更
しない。候補の時間対応・移動平均・不存在判定は、意図的にまだ行わない。
"""

import argparse
import csv
import hashlib
import json
import resource
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
import yaml

from evaluate_independent_depth_frames import recorded_transform, pose_for_footprint
from evaluate_single_depth_terrain import evaluate_points, messages, stamp_ns
from pm_evaluation.height_candidates import (
    CandidateOptions, extract_candidates, odom_planes, plane_reasons)
from pm_perception.depth_projection import sampled_points, transform_points
from pm_perception.mapper_replay_trace import load_mapper_trace


def counts(layer):
    """既知・黒・unknownを分離。既存RVizの100丸め基準を用いる。"""
    known = np.isfinite(layer)
    black = known & (np.rint(np.clip(layer, 0, 1) * 100) >= 100)
    return dict(known=int(known.sum()), black=int(black.sum()),
                unknown=int((~known).sum()))


def plot_frame(path, baseline, candidate):
    """選択frameだけ図化。候補をhazardと同一尺度で表示しない。"""
    panels = [('raw span [m]', baseline['cell_max'] - baseline['cell_min']),
              ('baseline reference hazard', baseline['reference_hazard']),
              ('candidate count', candidate['candidate_count'].astype(float)),
              ('broad groups', candidate['broad_count'].astype(float)),
              ('overflow groups', candidate['overflow_count'].astype(float)),
              ('above provisional plane hit', candidate['above_plane_hit'])]
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    for axis, (title, values) in zip(axes.flat, panels):
        values = values.copy()
        values[candidate['point_count'] == 0] = np.nan
        hazard = title == 'baseline reference hazard'
        cmap = plt.get_cmap('gray_r' if hazard else 'viridis').copy()
        cmap.set_bad('orchid')
        im = axis.imshow(values, origin='lower', cmap=cmap,
                         vmin=0, vmax=1 if hazard or title == 'above provisional plane hit' else None)
        axis.set_title(title)
        axis.set_xlabel('grid x'); axis.set_ylabel('grid y')
        fig.colorbar(im, ax=axis)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def prepare_frame(depth, frame, params):
    """同じ採用画素・TF・分散・相対原点で従来評価と候補用平面を準備する。"""
    if (depth.width != frame['width'] or depth.height != frame['height']
            or depth.header.frame_id != frame['image_frame']):
        raise ValueError('traceとdepthの解像度／frameが不一致です')
    optical, u, v, _ = sampled_points(depth, *frame['intrinsics'], params['pixel_stride'],
                                     params['min_depth'], params['max_depth'], return_pixels=True)
    points = transform_points(optical, recorded_transform(frame['camera_to_map']))
    base = frame['base_to_map']
    pose = pose_for_footprint(stamp_ns(depth), base['translation'] + base['quaternion'])
    offset = params['nominal_camera_height_above_ground'] - frame['camera_to_map']['translation'][2]
    baseline = evaluate_points(
        points, params, stamp_ns(depth), center=pose[1:3], heading_yaw=pose[-1],
        relative_elevation_offset=offset, observation_variance=params['measurement_variance']
        + params['depth_variance_per_meter_sq'] * np.sum(optical * optical, axis=1))
    return points, optical, u, v, pose, baseline, odom_planes(baseline, offset)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--mapper-trace', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--config', default=str(Path(__file__).resolve().parents[1]
                                              / 'config/height_candidate_diagnosis.yaml'))
    parser.add_argument('--every-seconds', type=float, default=0,
                        help='0ならtrace採用画像を全て評価。正値は動作確認用の間引き')
    parser.add_argument('--max-frames', type=int, default=0, help='0なら制限なし')
    parser.add_argument('--save-frames', type=int, default=3, help='図とNPZを保存する先頭frame数')
    args = parser.parse_args()
    if (not np.isfinite(args.every_seconds) or args.every_seconds < 0
            or args.max_frames < 0 or args.save_frames < 0):
        parser.error('frame数・secondsは有限の非負値')
    options = CandidateOptions(**yaml.safe_load(Path(args.config).read_text()))
    meta, frames, _ = load_mapper_trace(args.mapper_trace)
    params = dict(meta['parameters'])
    if params.get('feature_max_accepted_ground_age', 0) > 0:
        raise ValueError('ground受理時刻gateはこの独立frame診断で未対応です')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    # 採用stamp列から先に対象を決定。欠落画像を別stampの画像で補わない。
    selected = []
    for stamp in frames:
        if not selected or stamp - selected[-1] >= round(args.every_seconds * 1e9):
            selected.append(stamp)
        if args.max_frames and len(selected) >= args.max_frames:
            break
    wanted = set(selected)
    seen, frame_rows, times, baseline_times = set(), [], [], []
    totals = dict(observed_cells=0, candidates=0, sparse_groups=0, broad_groups=0,
                  overflow_groups=0, ground_provisional_cells=0, above_plane_hit_cells=0,
                  plane_known_cells=0, raw_obstacle_black_cells=0)
    with (output / 'candidates.csv').open('w', newline='') as stream, \
            (output / 'cells.csv').open('w', newline='') as cells_stream:
        writer = None
        cells_writer = None
        for _, _, record in messages(args.bag, [params['depth_topic']]):
            depth = deserialize_message(record.data, Image)
            stamp = stamp_ns(depth)
            if stamp not in wanted:
                continue
            if stamp in seen:
                raise ValueError('対象depth stampが重複しました')
            frame = frames[stamp]
            began = time.perf_counter()
            baseline_start = time.perf_counter()
            points, optical, u, v, pose, baseline, planes = prepare_frame(depth, frame, params)
            baseline_times.append(time.perf_counter() - baseline_start)
            # 従来cell最小zでfitした同一frame平面。真の地面と証明したモデルではない。
            # 非退化support・既存slope／roughness閾値内を診断基準のgateに用いる。
            reasons = plane_reasons(baseline, params)
            valid = reasons['plane_usable']
            candidate_start = time.perf_counter()
            candidates, layers = extract_candidates(
                points, np.column_stack((u, v)), baseline['origin'],
                baseline['hazard'].shape, params['resolution'], planes, valid,
                params['hazard_obstacle_height_limit'], options)
            elapsed = time.perf_counter() - candidate_start
            times.append(elapsed)
            layers.update(reasons)
            layers.update(support_count=baseline['support_count'], slope_deg=baseline['slope_deg'],
                          roughness=baseline['roughness'], plane_a=planes[:, :, 0],
                          plane_b=planes[:, :, 1], plane_c=planes[:, :, 2])
            origin_cell = np.rint(baseline['origin'] / params['resolution']).astype(int)
            for row in candidates:
                slot = row['slot']; width = baseline['hazard'].shape[1]
                row.update(stamp_ns=stamp, cell_x=int(origin_cell[0] + slot % width),
                           cell_y=int(origin_cell[1] + slot // width),
                           min_axial_depth_m=float(optical[row['min_point_index'], 2]),
                           max_axial_depth_m=float(optical[row['max_point_index'], 2]),
                           camera_distance_m=float(np.linalg.norm(
                               np.array([(row['x_lo_m'] + row['x_hi_m']) / 2,
                                         (row['y_lo_m'] + row['y_hi_m']) / 2,
                                         row['z_mean_m']]) - frame['camera_to_map']['translation'])))
                if writer is None:
                    writer = csv.DictWriter(stream, fieldnames=list(row))
                    writer.writeheader()
                writer.writerow(row)
            observed = layers['point_count'] > 0
            # 候補枠外のhitも元画素へ戻れるよう、観測cellごとの根拠をstream保存。
            for slot in np.flatnonzero(observed):
                row = dict(stamp_ns=stamp,
                           cell_x=int(origin_cell[0] + slot % baseline['hazard'].shape[1]),
                           cell_y=int(origin_cell[1] + slot // baseline['hazard'].shape[1]))
                for name, values in layers.items():
                    value = values.ravel()[slot].item()
                    row[name] = value if np.isfinite(value) else None
                for prefix in ('above_plane', 'body_height'):
                    index = int(layers[prefix + '_witness_index'].ravel()[slot])
                    row[prefix + '_u'] = int(u[index]) if index >= 0 else None
                    row[prefix + '_v'] = int(v[index]) if index >= 0 else None
                    row[prefix + '_axial_depth_m'] = float(optical[index, 2]) if index >= 0 else None
                    for axis, name in enumerate(('x', 'y', 'z')):
                        row[prefix + '_' + name + '_m'] = float(points[index, axis]) if index >= 0 else None
                if cells_writer is None:
                    cells_writer = csv.DictWriter(cells_stream, fieldnames=list(row))
                    cells_writer.writeheader()
                cells_writer.writerow(row)
            raw = baseline['cell_max'] - baseline['cell_min']
            raw_black = np.isfinite(raw) & (np.rint(np.clip(
                raw / params['hazard_obstacle_height_limit'], 0, 1) * 100) >= 100)
            metric = dict(observed_cells=int(observed.sum()),
                          candidates=int(layers['candidate_count'].sum()),
                          sparse_groups=int(layers['sparse_count'].sum()),
                          broad_groups=int(layers['broad_count'].sum()),
                          overflow_groups=int(layers['overflow_count'].sum()),
                          ground_provisional_cells=int(layers['ground_provisional'].sum()),
                          above_plane_hit_cells=int(np.nansum(layers['above_plane_hit'])),
                          plane_known_cells=int(np.isfinite(layers['above_plane_hit']).sum()),
                          raw_obstacle_black_cells=int(raw_black.sum()))
            for k in totals:
                totals[k] += metric[k]
            frame_rows.append(dict(stamp_ns=stamp, points=len(points), candidate_ms=elapsed*1000,
                                   frame_total_ms=(time.perf_counter()-began)*1000,
                                   baseline_reference_hazard=counts(baseline['reference_hazard']),
                                   **metric))
            if len(seen) < args.save_frames:
                np.savez_compressed(output / ('frame_%d.npz' % stamp),
                                    **layers, origin=baseline['origin'],
                                    baseline_reference_hazard=baseline['reference_hazard'],
                                    baseline_cell_min=baseline['cell_min'],
                                    baseline_cell_max=baseline['cell_max'])
                plot_frame(output / ('frame_%d.png' % stamp), baseline, layers)
            seen.add(stamp)
            if len(seen) == len(wanted):
                break
    missing = sorted(wanted - seen)
    if missing:
        raise ValueError('%d対象画像がbagにありません。完了summaryは作りません' % len(missing))

    def timing(values):
        milliseconds = np.asarray(values) * 1000
        return dict(mean_ms=float(milliseconds.mean()), p95_ms=float(np.percentile(milliseconds, 95)),
                    max_ms=float(milliseconds.max()))

    summary = dict(schema=1, bag=str(Path(args.bag).resolve()),
                   trace=str(Path(args.mapper_trace).resolve()),
                   trace_sha256=hashlib.sha256(Path(args.mapper_trace).read_bytes()).hexdigest(),
                   parameters=params, candidate_options=vars(options), selected_frames=len(seen),
                   total_trace_frames=len(frames), sampling_every_seconds=args.every_seconds,
                   temporal_fusion=False, moving_average=False, existence_update=False,
                   calibrated_existence_probability=False, ground_source='provisional baseline frame plane',
                   hazard_replaced=False, qos='offline file read; no ROS subscription',
                   candidate_timing=timing(times), baseline_timing=timing(baseline_times),
                   peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                   totals=totals)
    # 同じtraceでもsampling helperの改訂で採用画素が変わるため、使用sourceを識別。
    sources = [Path(__file__).resolve()] + [Path(sys.modules[name].__file__).resolve()
               for name in ('pm_evaluation.height_candidates', 'pm_perception.depth_projection',
                            'pm_perception.terrain_features', 'evaluate_single_depth_terrain')]
    summary['source_sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in sources}
    (output/'frames.json').write_text(json.dumps(frame_rows, ensure_ascii=False, indent=2)+'\n')
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
