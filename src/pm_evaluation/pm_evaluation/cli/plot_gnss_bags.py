"""bagのGNSS軌跡だけを既存OSM画像へ重ね、開始・終了点を示す。"""
import argparse
from bisect import bisect_left
from collections import Counter
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages
import numpy as np
from pyproj import Geod, Transformer

from pm_evaluation.cli.plot_bag_trajectories import (
    GNSS_STATE_STYLE, find_mcap_files, infer_rtk_state_from_msg,
)


def nearest_state(states, t):
    """bag時刻で近いNAV-PVTを探す。2秒超離れた状態は流用しない。"""
    if not states:
        return 'UNKNOWN'
    i = bisect_left([v[0] for v in states], t)
    candidates = states[max(0, i-1):min(len(states), i+1)]
    nearest = min(candidates, key=lambda v: abs(v[0]-t))
    return nearest[1] if abs(nearest[0]-t) <= 2_000_000_000 else 'UNKNOWN'


def read_inputs(bag):
    """センサheaderではなく元bag記録時刻でGNSSと測位状態を対応させる。"""
    fixes, states = [], []
    for path in find_mcap_files(bag):
        for item in read_ros2_messages(path, topics=['/fix', '/navpvt'], log_time_order=True):
            msg = item.ros_msg
            if item.channel.topic == '/navpvt':
                states.append((int(item.log_time_ns), infer_rtk_state_from_msg(msg)))
            elif (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
                  and -90 <= msg.latitude <= 90 and -180 <= msg.longitude <= 180
                  and msg.status.status >= 0):
                fixes.append((int(item.log_time_ns), float(msg.longitude), float(msg.latitude)))
    fixes.sort(); states.sort()
    return fixes, states


def plot_one(bag, basemap, overwrite=False):
    fixes, states = read_inputs(bag)
    if not fixes:
        print('{}: 有効な/fixがないため図を作成できません'.format(bag.name), flush=True)
        return {'bag': bag.name, 'status': 'no_valid_fix'}
    start = fixes[0]
    geod = Geod(ellps='WGS84')
    def enu(lon, lat):
        az, _, distance = geod.inv(start[1], start[2], lon, lat)
        return distance * math.sin(math.radians(az)), distance * math.cos(math.radians(az))
    points = [(t, *enu(lon,lat), nearest_state(states, t)) for t,lon,lat in fixes]
    background = np.load(basemap)
    bounds = background['extent']
    transform = Transformer.from_crs('EPSG:3857', 'EPSG:4326', always_xy=True)
    lower = enu(*transform.transform(bounds[0],bounds[2]))
    upper = enu(*transform.transform(bounds[1],bounds[3]))
    map_extent = [lower[0],upper[0],lower[1],upper[1]]
    xs, ys = [p[1] for p in points], [p[2] for p in points]
    # OSMキャッシュがコースを覆わない場合は、地図付きと誤認される結果を残さない。
    if min(xs) < map_extent[0] or max(xs) > map_extent[1] or min(ys) < map_extent[2] or max(ys) > map_extent[3]:
        raise RuntimeError('{}のGNSSが指定地図キャッシュの範囲外'.format(bag.name))
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.imshow(background['image'], extent=map_extent, origin='upper', zorder=0)
    # GNSS欠測が3秒を超える部分は直線でつながない。
    last = None
    for point in points:
        if last is not None and point[0]-last[0] <= 3_000_000_000:
            ax.plot([last[1],point[1]], [last[2],point[2]], color='black', alpha=.35, lw=1, zorder=1)
        last = point
    # 同じ地点を通る周回でも時系列の向きが分かるよう、数か所だけ矢印を置く。
    for fraction in (.2, .4, .6, .8):
        i = min(len(points)-1, int(fraction * (len(points)-1)))
        j = min(len(points)-1, i+4)
        if j > i and points[j][0]-points[i][0] <= 6_000_000_000 and math.hypot(points[j][1]-points[i][1], points[j][2]-points[i][2]) > 1.:
            ax.annotate('', xy=(points[j][1], points[j][2]), xytext=(points[i][1], points[i][2]),
                        arrowprops={'arrowstyle':'->','color':'black','lw':1.5}, zorder=5)
    for state, style in GNSS_STATE_STYLE.items():
        selected = [p for p in points if p[3] == state]
        if selected:
            ax.scatter([p[1] for p in selected], [p[2] for p in selected], s=18,
                       c=style['color'], marker=style['marker'], label=style['label'], zorder=3)
    # 起終点が同一地点でも確認できるよう、異なる記号と反対方向の注釈を併用する。
    ax.scatter([points[0][1]], [points[0][2]], marker='^', s=150, c='limegreen',
               edgecolors='black', linewidths=1.3, zorder=6, label='Start (first valid fix)')
    ax.scatter([points[-1][1]], [points[-1][2]], marker='X', s=140, c='crimson',
               edgecolors='black', linewidths=1.3, zorder=7, label='Goal (last valid fix)')
    ax.annotate('START', (points[0][1],points[0][2]), xytext=(12,18), textcoords='offset points',
                arrowprops={'arrowstyle':'->','color':'green'}, fontsize=10, fontweight='bold', color='darkgreen')
    ax.annotate('GOAL', (points[-1][1],points[-1][2]), xytext=(-48,-24), textcoords='offset points',
                arrowprops={'arrowstyle':'->','color':'darkred'}, fontsize=10, fontweight='bold', color='darkred')
    dx=max(xs)-min(xs); dy=max(ys)-min(ys); margin=max(8., max(dx,dy)*.12)
    ax.set_xlim(min(xs)-margin,max(xs)+margin); ax.set_ylim(min(ys)-margin,max(ys)+margin)
    ax.set_aspect('equal',adjustable='box'); ax.grid(alpha=.25)
    ax.set(xlabel='East [m]', ylabel='North [m]', title=bag.name+' — GNSS trajectory')
    ax.legend(loc='best', fontsize=8)
    ax.text(.99,.01,'Map: © OpenStreetMap contributors', transform=ax.transAxes, ha='right',fontsize=8,
            bbox={'facecolor':'white','alpha':.8,'edgecolor':'none'})
    fig.tight_layout()
    target=bag/'gnss_trajectory_osm.png'
    if target.exists() and not overwrite:
        raise RuntimeError('既存画像を上書きしません: {}'.format(target))
    fig.savefig(target,dpi=180); plt.close(fig)
    distance=sum(geod.inv(a[1],a[2],b[1],b[2])[2] for a,b in zip(fixes, fixes[1:]) if b[0]-a[0] <= 3_000_000_000)
    return {'bag':bag.name,'status':'ok','fix_count':len(points),
            'duration_s':(points[-1][0]-points[0][0])*1e-9,
            'gnss_path_length_m':distance,'start_goal_distance_m':math.hypot(points[-1][1],points[-1][2]),
            'states':dict(Counter(p[3] for p in points)), 'image':str(target)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags',nargs='+',type=Path)
    parser.add_argument('--basemap',type=Path,required=True)
    parser.add_argument('--summary',type=Path,required=True)
    parser.add_argument('--overwrite', action='store_true', help='同名の既存図を上書きする')
    args=parser.parse_args()
    results=[]
    for bag in args.bags:
        try:
            results.append(plot_one(bag,args.basemap,args.overwrite))
        except Exception as error:
            results.append({'bag':bag.name,'status':'error','reason':str(error)})
            print('{}: {}'.format(bag.name,error), flush=True)
    args.summary.write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(results,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
