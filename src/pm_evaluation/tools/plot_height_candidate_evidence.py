#!/usr/bin/env python3
"""unknown理由・overflowの代表セルをdepthとRGBに照合するオフライン図化。

RGBは最も近い撮像時刻の文脈画像で、depthとの画素対応を主張しない。
少数対象だけ元bagを再走査し、画像全列はメモリへ保持しない。
"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

from diagnose_height_candidates import prepare_frame
from evaluate_single_depth_terrain import messages, stamp_ns
from pm_perception.mapper_replay_trace import load_mapper_trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('diagnosis')
    parser.add_argument('--bag', required=True)
    parser.add_argument('--mapper-trace', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    folder = Path(args.output)
    folder.mkdir(parents=True, exist_ok=False)
    meta, frames, _ = load_mapper_trace(args.mapper_trace)
    params = meta['parameters']
    with (Path(args.diagnosis)/'cells.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    categories = {'support': lambda r: int(r['plane_reason_bits']) == 2,
                  'slope': lambda r: int(r['plane_reason_bits']) == 8,
                  'slope_roughness': lambda r: int(r['plane_reason_bits']) == 24,
                  'overflow': lambda r: int(r['overflow_count']) > 0,
                  'accepted_high': lambda r: r['above_plane_hit'] == '1.0'}
    targets = []
    for name, condition in categories.items():
        candidates = [r for r in rows if condition(r) and float(r['raw_span']) >= .08]
        if candidates:
            target = max(candidates, key=lambda r: float(r['raw_span']))
            targets.append(dict(target, category=name))
    stamps = set(int(r['stamp_ns']) for r in targets)
    colors, depths = {}, {}
    for _, channel, record in messages(args.bag, [params['depth_topic'], '/oak/color/image_raw']):
        message = deserialize_message(record.data, Image)
        stamp = stamp_ns(message)
        if channel.topic == params['depth_topic'] and stamp in stamps:
            depths[stamp] = message
        elif channel.topic == '/oak/color/image_raw':
            for target in stamps:
                if target not in colors or abs(stamp-target) < abs(stamp_ns(colors[target])-target):
                    colors[target] = message
    summaries = []
    for target in targets:
        stamp = int(target['stamp_ns']); frame = frames[stamp]; depth = depths[stamp]
        points, optical, u, v, _, baseline, _ = prepare_frame(depth, frame, params)
        origin = np.rint(baseline['origin']/params['resolution']).astype(int)
        cell = np.array([int(target['cell_x']), int(target['cell_y'])])
        xy = np.floor(points[:, :2]/params['resolution']).astype(int)
        selected = (xy == cell).all(axis=1)
        nearby = (np.abs(xy-cell) <= 1).all(axis=1)
        indices = np.flatnonzero(selected)
        raw = np.frombuffer(depth.data, dtype='>u2' if depth.is_bigendian else '<u2').reshape(
            depth.height, depth.step//2)[:, :depth.width] * .001
        y, x = (cell-origin)[::-1]
        fig, axes = plt.subplots(2, 3, figsize=(16, 9))
        color = colors.get(stamp)
        if color is not None and color.encoding in ('rgb8', 'bgr8'):
            rgb = np.frombuffer(color.data, dtype=np.uint8).reshape(color.height, color.step)
            rgb = rgb[:, :color.width*3].reshape(color.height, color.width, 3)
            axes[0, 0].imshow(rgb if color.encoding == 'rgb8' else rgb[:, :, ::-1])
        axes[0, 0].set_title('RGB context, not pixel aligned')
        axes[0, 1].imshow(np.where(raw > 0, raw, np.nan), vmin=.4, vmax=5)
        axes[0, 1].scatter(u[selected], v[selected], s=10, c='red')
        axes[0, 1].set_title('depth [m]; target cell pixels in red')
        axes[0, 2].hist(points[selected, 2], bins=min(20, max(1, len(indices))))
        axes[0, 2].set_title('target observed z distribution [m]')
        axes[1, 0].imshow(baseline['cell_min'], origin='lower')
        axes[1, 0].scatter([x], [y], c='red', s=30)
        axes[1, 0].set_title('frame cell minimum z [m]')
        axes[1, 1].scatter(points[nearby, 0], points[nearby, 2], s=8)
        axes[1, 1].scatter(points[selected, 0], points[selected, 2], c='red', s=10)
        axes[1, 1].set_title('3x3 support points, x-z [m]')
        axes[1, 2].axis('off')
        axes[1, 2].text(0, .9, '\n'.join('%s: %s' % (k, target[k]) for k in
            ('category', 'cell_x', 'cell_y', 'raw_span', 'point_count', 'support_count',
             'slope_deg', 'roughness', 'plane_reason_bits', 'overflow_count')), va='top')
        fig.tight_layout()
        name = target['category'] + '_' + str(stamp)
        fig.savefig(folder/(name+'.png'), dpi=120); plt.close(fig)
        evidence = dict(target, color_stamp_ns=stamp_ns(color) if color else None,
                        color_time_difference_seconds=(stamp_ns(color)-stamp)/1e9 if color else None)
        for label, index in (('min', indices[np.argmin(points[selected, 2])]),
                             ('max', indices[np.argmax(points[selected, 2])])):
            patch = raw[max(0, v[index]-1):v[index]+2, max(0, u[index]-1):u[index]+2]
            values = patch[(patch >= params['min_depth']) & (patch <= params['max_depth'])]
            evidence[label+'_pixel'] = [int(u[index]), int(v[index])]
            evidence[label+'_depth_m'] = float(optical[index, 2])
            evidence[label+'_patch_median_m'] = float(np.median(values)) if len(values) else None
            evidence[label+'_point_xyz_m'] = points[index].tolist()
        np.savez_compressed(folder/(name+'.npz'), points=points[selected],
                            uv=np.column_stack((u[selected], v[selected])),
                            axial_depth=optical[selected, 2], nearby_points=points[nearby])
        summaries.append(evidence)
    (folder/'evidence.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2)+'\n')
    print('saved %d cases' % len(summaries))


if __name__ == '__main__':
    main()
