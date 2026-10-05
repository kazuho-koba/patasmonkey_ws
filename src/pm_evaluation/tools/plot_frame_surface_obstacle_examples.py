#!/usr/bin/env python3
"""代表黒cellの元depth画素と同cellの選択点高さを図示する。

RGBとの厳密対応や物体同定は行わない。元画像のdepthと投影された高さ分布を
見比べ、単一点・別表面・縦方向に離れたpixelの混在を追うための補助図。
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
from evaluate_single_depth_terrain import messages, stamp_ns
from evaluate_independent_depth_frames import recorded_transform
from pm_perception.depth_projection import sampled_points, transform_points
from pm_perception.mapper_replay_trace import load_mapper_trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--mapper-trace', required=True)
    args = parser.parse_args()
    summary = json.loads((args.source/'summary.json').read_text()); p = summary['parameters']
    with (args.source/'black_cell_evidence.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    # 分位点で大きく変わる例と、傾斜差引きで閾値を下回る近平坦plane例を抽出。
    quantile = [r for r in rows if float(r['quantile_span']) < p['hazard_obstacle_height_limit']*.995]
    plane = [r for r in rows if float(r['detrended_span']) < p['hazard_obstacle_height_limit']*.995
             and float(r['plane_slope_deg']) < 20]
    selected = [(name,max(group,key=lambda r:float(r['raw_span']))) for name,group in
                (('quantile_example',quantile),('plane_example',plane)) if group]
    _, frames, _ = load_mapper_trace(args.mapper_trace)
    targets = {int(r['stamp_ns']):(name,r) for name,r in selected}
    for _,_,record in messages(args.bag,[p['depth_topic']]):
        message = deserialize_message(record.data,Image); stamp = stamp_ns(message)
        if stamp not in targets:
            continue
        name,row = targets.pop(stamp); frame = frames[stamp]
        raw = np.frombuffer(message.data,dtype='>u2' if message.is_bigendian else '<u2').reshape(
            message.height,message.step//2)[:,:message.width].astype(float)*.001
        optical,u,v,_ = sampled_points(message,*frame['intrinsics'],summary['stride'],p['min_depth'],p['max_depth'],return_pixels=True)
        points = transform_points(optical,recorded_transform(frame['camera_to_map']))
        xy = np.floor(points[:,:2]/p['resolution']).astype(int)
        cell = (int(row['cell_x']),int(row['cell_y']))
        inside = (xy[:,0] == cell[0]) & (xy[:,1] == cell[1])
        fig,axes = plt.subplots(1,2,figsize=(12,4))
        valid = (raw >= p['min_depth']) & (raw <= p['max_depth']) & (raw > 0)
        axes[0].imshow(np.where(valid,raw,np.nan),vmin=p['min_depth'],vmax=p['max_depth'],cmap='viridis')
        axes[0].scatter(u[inside],v[inside],s=12,facecolors='none',edgecolors='orange')
        for label,color in (('min','cyan'),('max','red')):
            axes[0].scatter(int(row[label+'_u']),int(row[label+'_v']),c=color,s=50,marker='x',label=label)
        axes[0].legend(); axes[0].set_title('Depth[m], selected pixels in odom cell')
        axes[1].hist(points[inside,2],bins=min(20,max(3,int(inside.sum()))))
        axes[1].set_xlabel('odom z [m]'); axes[1].set_ylabel('selected pixel count')
        axes[1].set_title('raw span %.3fm / q90-q10 %.3fm'% (float(row['raw_span']),float(row['quantile_span'])))
        fig.suptitle('stamp %d, odom cell %s, stride %d'%(stamp,cell,summary['stride']))
        fig.tight_layout(); fig.savefig(args.source/(name+'.png'),dpi=150); plt.close(fig)
        if not targets:
            break
    if targets:
        raise ValueError('対象stampがbagにない')
    # RGBは撮像stampが最も近い画像を別図にする。depth/RGBの画素位置を同一視
    # しない（intrinsics/extrinsicsに基づく重畳はこの診断の対象外）。
    nearest = {}
    for _,_,record in messages(args.bag,['/oak/color/image_raw']):
        message = deserialize_message(record.data,Image); stamp = stamp_ns(message)
        for name,row in selected:
            delta = abs(stamp-int(row['stamp_ns']))
            if name not in nearest or delta < nearest[name][0]:
                if message.encoding not in ('rgb8','bgr8'):
                    raise ValueError('未対応RGB encoding: '+message.encoding)
                rgb = np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)[:,:message.width*3].reshape(message.height,message.width,3)
                if message.encoding == 'bgr8':
                    rgb = rgb[:,:,::-1]
                nearest[name] = (delta,stamp,rgb.copy())
    for name,(delta,stamp,rgb) in nearest.items():
        fig,ax = plt.subplots(figsize=(8,5))
        ax.imshow(rgb); ax.axis('off')
        ax.set_title('Nearest RGB stamp %d, depth offset %.1f ms (no pixel alignment)'%(stamp,delta/1e6))
        fig.tight_layout(); fig.savefig(args.source/(name+'_rgb.png'),dpi=150); plt.close(fig)


if __name__ == '__main__':
    main()
