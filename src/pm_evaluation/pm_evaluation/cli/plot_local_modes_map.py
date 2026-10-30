"""再計算したlocal方式をGNSS付近の地図と高さ時系列に描画する。"""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcap.reader import make_reader
import numpy as np
from pyproj import Geod, Transformer
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay_dir', type=Path)
    parser.add_argument('input_bag', type=Path)
    parser.add_argument('--basemap', type=Path, required=True)
    parser.add_argument('--seed-stamp', type=float, required=True)
    parser.add_argument('--settle-sec', type=float, default=.5)
    args = parser.parse_args()
    begin = args.seed_stamp + args.settle_sec
    modes = [('legacy', 'Legacy'), ('separated', 'Separated'), ('projected', 'Projected 2D'),
             ('ekf3d_no_accel', '3D no acceleration'), ('ekf3d_accel', '3D with acceleration')]
    data = {}
    for name, label in modes:
        with (args.replay_dir/(name+'.csv')).open() as f:
            rows = [{k: float(v) for k,v in row.items()} for row in csv.DictReader(f)]
        rows = sorted((r for r in rows if r['stamp'] >= begin), key=lambda r:r['stamp'])
        if not rows:
            raise RuntimeError('{}: 初期化後のodometryがありません'.format(name))
        data[name] = rows
    fixes = []
    for path in sorted(args.input_bag.glob('*.mcap')):
        with path.open('rb') as f:
            for _,_,record in make_reader(f).iter_messages(topics=['/fix']):
                msg = deserialize_message(record.data, NavSatFix)
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                if msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
                    fixes.append((t, msg.longitude, msg.latitude))
    if not fixes:
        raise RuntimeError('有効なGNSS fixがありません')
    anchor = min(fixes, key=lambda p:abs(p[0]-begin))
    geod = Geod(ellps='WGS84')
    def enu(lon,lat):
        az,_,distance = geod.inv(anchor[1], anchor[2], lon, lat)
        return distance*math.sin(math.radians(az)), distance*math.cos(math.radians(az))
    gps = [(*enu(lon,lat),t) for t,lon,lat in fixes if t >= begin]
    background = np.load(args.basemap)
    extent = background['extent']
    mercator = Transformer.from_crs('EPSG:3857', 'EPSG:4326', always_xy=True)
    lo = enu(*mercator.transform(extent[0],extent[2]))
    hi = enu(*mercator.transform(extent[1],extent[3]))
    image_extent = [lo[0], hi[0], lo[1], hi[1]]
    converted = {}
    for name,label in modes:
        # 初期位置のみ同一GNSSへ平行移動。以降は回転・スケールfitを一切しない。
        rows = data[name]; x0, y0 = rows[0]['x'], rows[0]['y']
        converted[name] = [(r['x']-x0,r['y']-y0,r['z'],r['stamp']) for r in rows]
        with (args.replay_dir/(name+'_map.csv')).open('w',newline='') as f:
            writer=csv.writer(f); writer.writerow(['east_m','north_m','z_m','stamp']); writer.writerows(converted[name])
    accepted_height_stamps = []
    for path in sorted((args.replay_dir/'replayed_bag').glob('*.mcap')):
        with path.open('rb') as f:
            for _,_,record in make_reader(f).iter_messages(topics=['/vio/odometry/gated']):
                msg = deserialize_message(record.data, Odometry)
                accepted_height_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
    last_height = max(accepted_height_stamps) if accepted_height_stamps else None
    with (args.replay_dir/'gnss_enu.csv').open('w',newline='') as f:
        writer=csv.writer(f); writer.writerow(['east_m','north_m','stamp']); writer.writerows(gps)
    comparisons = {}
    for name,label in modes:
        points = converted[name]
        ts = np.array([p[3] for p in points]); xs = np.array([p[0] for p in points]); ys = np.array([p[1] for p in points])
        distances = []
        for gx,gy,t in gps:
            index = int(np.searchsorted(ts, t))
            if index == 0 or index >= len(ts) or ts[index]-ts[index-1] > 1.0:
                continue
            x = float(np.interp(t, ts, xs)); y = float(np.interp(t, ts, ys))
            distances.append(math.hypot(x-gx,y-gy))
        comparisons[name] = {'n':len(distances), 'median_m':float(np.median(distances)) if distances else None,
                             'p95_m':float(np.percentile(distances,95)) if distances else None}
    (args.replay_dir/'gnss_comparison.json').write_text(json.dumps(comparisons,ensure_ascii=False,indent=2)+'\n')
    fig,ax = plt.subplots(figsize=(10,9))
    ax.imshow(background['image'], extent=image_extent, origin='upper')
    ax.plot([p[0] for p in gps],[p[1] for p in gps],color='black',linewidth=1.4,label='GNSS /fix')
    for name,label in modes:
        pts=converted[name]
        ax.plot([p[0] for p in pts],[p[1] for p in pts],label=label,linewidth=1.2)
    ax.scatter([0],[0],marker='o',color='green',label='GNSS anchor')
    ax.set_aspect('equal',adjustable='box'); ax.grid(alpha=.25)
    ax.set(xlabel='East [m]',ylabel='North [m]',title='Five local odometries — magnetic seed, no heading fit')
    ax.legend(fontsize=8)
    ax.text(.99,.01,'Map: © OpenStreetMap contributors',transform=ax.transAxes,ha='right',fontsize=8)
    fig.tight_layout(); fig.savefig(args.replay_dir/'five_modes_map_full.png',dpi=170)
    xx=[p[0] for p in gps]; yy=[p[1] for p in gps]
    ax.set_xlim(min(xx)-20,max(xx)+20); ax.set_ylim(min(yy)-20,max(yy)+20)
    ax.set_title('Five local odometries — GNSS area')
    fig.savefig(args.replay_dir/'five_modes_map_gnss_area.png',dpi=170); plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,5))
    for name,label in modes:
        rows=data[name]
        ax.plot([r['stamp']-begin for r in rows],[r['z'] for r in rows],label=label,linewidth=1)
    if last_height is not None and last_height >= begin:
        ax.axvline(last_height-begin, color='black', linestyle='--', alpha=.7,
                   label='Last accepted VIO z')
    ax.set(xlabel='time after heading seed [s]',ylabel='local z [m]',title='Height from each estimator')
    ax.grid(); ax.legend(); fig.tight_layout(); fig.savefig(args.replay_dir/'five_modes_height.png',dpi=170); plt.close(fig)
    # 発散条件がある場合でも残りの高さ変化を確認できるよう、個別の縦軸も用意する。
    fig, axes = plt.subplots(len(modes), 1, figsize=(12, 11), sharex=True)
    for axis, (name, label) in zip(axes, modes):
        rows = data[name]
        axis.plot([r['stamp']-begin for r in rows], [r['z'] for r in rows], linewidth=1)
        if last_height is not None and last_height >= begin:
            axis.axvline(last_height-begin, color='black', linestyle='--', alpha=.7)
        axis.set_ylabel('z [m]'); axis.set_title(label, loc='left'); axis.grid()
    axes[-1].set_xlabel('time after heading seed [s]')
    fig.tight_layout(); fig.savefig(args.replay_dir/'five_modes_height_individual.png',dpi=150); plt.close(fig)
    (args.replay_dir/'map_alignment.json').write_text(json.dumps({'seed_stamp':args.seed_stamp,'plot_start_stamp':begin,
       'gnss_anchor_time':anchor[0], 'gnss_anchor_lon':anchor[1], 'gnss_anchor_lat':anchor[2],
       'last_accepted_vio_z_stamp':last_height,
       'alignment':'初期GNSS位置へ平行移動のみ。軌跡の回転fit・縮尺調整なし。GNSSは真値ではない。'},ensure_ascii=False,indent=2)+'\n')

if __name__ == '__main__':
    main()
