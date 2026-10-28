"""再計算したlocal/global EKFとGNSS・記録済みVIOを地図へ重ねる。"""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcap.reader import make_reader
import numpy as np
from pyproj import Geod, Transformer
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.serialization import deserialize_message


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def read_replay(bag):
    """bagのwall record時刻でREADYを特定し、軌跡は各header時刻で対応付ける。"""
    types={'/fix':NavSatFix, '/fix/gated':NavSatFix,
           '/vio/odometry':Odometry, '/odometry/local':Odometry,
           '/odometry/global':Odometry, '/odometry/gps':Odometry,
           '/localization/initialization/status':String,
           '/vio/twist_gate/diagnostics':DiagnosticArray,
           '/vio/vertical_gate/diagnostics':DiagnosticArray,
           '/gnss/fix_gate/diagnostics':DiagnosticArray}
    result={topic:[] for topic in types}
    for path in sorted(bag.glob('*.mcap')):
        with path.open('rb') as stream:
            for _,channel,record in make_reader(stream).iter_messages(topics=list(types)):
                topic=channel.topic
                msg=deserialize_message(record.data,types[topic])
                wall=record.log_time*1e-9
                if topic.endswith('/status'):
                    try:
                        value=json.loads(msg.data)
                    except ValueError:
                        continue
                    result[topic].append({'wall':wall, **value})
                    continue
                if topic.endswith('/diagnostics'):
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                    for diagnostic in msg.status:
                        values={v.key:v.value for v in diagnostic.values}
                        result[topic].append(dict(t=stamp,wall=wall,name=diagnostic.name,
                                                  level=diagnostic.level,message=diagnostic.message,
                                                  state=values.get('state','')))
                    continue
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                if topic in ('/fix','/fix/gated'):
                    if msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
                        result[topic].append(dict(t=stamp,wall=wall,lat=msg.latitude,lon=msg.longitude))
                else:
                    p=msg.pose.pose.position
                    result[topic].append(dict(t=stamp,wall=wall,x=p.x,y=p.y,z=p.z,
                                              yaw=yaw(msg.pose.pose.orientation)))
    return result


def nearest(rows, t, key='t'):
    return min(rows, key=lambda row:abs(row[key]-t))


def write_csv(path, rows, fields):
    with path.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay_dir',type=Path)
    parser.add_argument('--basemap',type=Path,required=True,
                        help='同じ領域のWeb Mercator背景画像npz（image, extent）')
    args=parser.parse_args()
    data=read_replay(args.replay_dir/'replayed_bag')
    ready=next((row for row in data['/localization/initialization/status']
                if row.get('state')=='READY'),None)
    if ready is None:
        raise SystemExit('READYが記録されていないため、地図上の初期整列を行えません')
    locals_=sorted((r for r in data['/odometry/local'] if r['wall'] >= ready['wall']),key=lambda r:r['t'])
    globals_=sorted((r for r in data['/odometry/global'] if r['wall'] >= ready['wall']),key=lambda r:r['t'])
    fixes=data['/fix']
    gated=[r for r in data['/fix/gated'] if r['wall'] <= ready['wall']]
    vio=sorted(data['/vio/odometry'],key=lambda r:r['t'])
    if not all((locals_,globals_,fixes,gated,vio)):
        raise SystemExit('初期化後のlocal/global、GNSS、VIOのいずれかが欠落しています')
    origin=gated[-1]
    geod=Geod(ellps='WGS84')
    def geo_xy(lon,lat):
        az,_,dist=geod.inv(origin['lon'],origin['lat'],lon,lat)
        return dist*math.sin(math.radians(az)),dist*math.cos(math.radians(az))
    gnss=[]
    for f in fixes:
        x,y=geo_xy(f['lon'],f['lat'])
        gnss.append({**f,'x':x,'y':y})
    # 同一datumを基点とする両EKFは回転も縮尺も変えない。GNSSアンテナとbaseの
    # 取付差は数cm程度残るため、ここで軌跡fitや回転補正は行わない。
    base=locals_[0]
    trajectories={}
    for name,rows in [('local',locals_),('global',globals_)]:
        trajectories[name]=[{**r,'x':r['x']-base['x'],'y':r['y']-base['y']} for r in rows]
    # 記録VIOは独自のodom原点とyawを持つ。READY時のVIO yawとlocal EKF yawを
    # 一度だけ合わせ、以降の軌跡にはfit・拡大縮小を一切しない。
    ref=nearest(vio,base['t'])
    angle=base['yaw']-ref['yaw']
    c,s=math.cos(angle),math.sin(angle)
    trajectories['vio']=[]
    vio_all=[]
    for row in vio:
        dx,dy=row['x']-ref['x'],row['y']-ref['y']
        transformed={**row,'x':c*dx-s*dy,'y':s*dx+c*dy}
        vio_all.append(transformed)
        if row['t'] >= base['t']:
            trajectories['vio'].append(transformed)
    if not trajectories['vio']:
        raise SystemExit('初期化後のVIOがありません')
    write_csv(args.replay_dir/'gnss_enu.csv',gnss,['t','wall','lat','lon','x','y'])
    for name,rows in trajectories.items():
        write_csv(args.replay_dir/(name+'_enu.csv'),rows,['t','wall','x','y','z','yaw'])
    write_csv(args.replay_dir/'vio_all_recorded_enu.csv',vio_all,['t','wall','x','y','z','yaw'])
    # Mercator背景の四隅をdatum基準の局所東北[m]に変換する。小さな表示範囲では
    # 仮定した平面内の線形縮尺誤差は微小であり、固定cos補正の誤用を避ける。
    background=np.load(args.basemap)
    # レポートと同じフォルダに背景を残し、後日同じ領域で再描画できるようにする。
    if args.basemap.resolve() != (args.replay_dir/'basemap.npz').resolve():
        shutil.copy2(args.basemap, args.replay_dir/'basemap.npz')
    extent=background['extent']
    transform=Transformer.from_crs('EPSG:3857','EPSG:4326',always_xy=True)
    lo=geo_xy(*transform.transform(extent[0],extent[2]))
    hi=geo_xy(*transform.transform(extent[1],extent[3]))
    image_extent=[lo[0],hi[0],lo[1],hi[1]]
    gx=np.array([r['x'] for r in gnss]); gy=np.array([r['y'] for r in gnss])
    margin=20.
    bounds=[float(gx.min()-margin),float(gx.max()+margin),
            float(gy.min()-margin),float(gy.max()+margin)]
    colors={'vio':'tab:orange','local':'tab:blue','global':'tab:red'}
    labels={'vio':'Recorded VIO (initial yaw aligned; divergent)',
            'local':'Recomputed local EKF','global':'Recomputed global EKF'}
    def figure(name, selected, full=False):
        fig,ax=plt.subplots(figsize=(9,8))
        ax.imshow(background['image'],extent=image_extent,origin='upper',zorder=0)
        ax.plot(gx,gy,color='black',linewidth=1,label='GNSS /fix',zorder=2)
        ax.scatter(gx,gy,s=6,color='black',alpha=.45,zorder=2)
        ax.scatter([0],[0],s=60,color='green',marker='o',label='initial GNSS datum',zorder=5)
        allx=[gx];ally=[gy]
        for key in selected:
            rows=trajectories[key]
            x=np.array([r['x'] for r in rows]);y=np.array([r['y'] for r in rows])
            ax.plot(x,y,color=colors[key],linewidth=1.2,label=labels[key],zorder=3)
            allx.append(x);ally.append(y)
        if full:
            xx=np.concatenate(allx);yy=np.concatenate(ally)
            ax.set_xlim(xx.min()-margin,xx.max()+margin);ax.set_ylim(yy.min()-margin,yy.max()+margin)
        else:
            ax.set_xlim(bounds[0],bounds[1]);ax.set_ylim(bounds[2],bounds[3])
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.25)
        ax.set_xlabel('East [m]');ax.set_ylabel('North [m]')
        ax.set_title(name + (' — GNSS area' if not full else ' — all extents'))
        ax.legend(loc='best',fontsize=8)
        ax.text(.99,.01,'Map: © OpenStreetMap contributors',transform=ax.transAxes,
                ha='right',va='bottom',fontsize=7,
                bbox=dict(facecolor='white',alpha=.75,edgecolor='none'))
        fig.tight_layout()
        fig.savefig(args.replay_dir/(name.replace(' ','_').lower()+('.png' if not full else '_full.png')),dpi=170)
        plt.close(fig)
    figure('GNSS',[])
    for key,title in [('vio','VO'),('local','Local EKF'),('global','Global EKF')]:
        figure(title,[key])
    figure('All trajectories',['vio','local','global'])
    figure('VO',['vio'],True)
    figure('All trajectories',['vio','local','global'],True)
    # VIOが初期化以前から破綻している場合も省かず、全記録区間を別途表示する。
    after_ready=trajectories['vio']
    trajectories['vio']=vio_all
    figure('VO all recorded',['vio'],True)
    trajectories['vio']=after_ready
    # GNSSは真値とせず、同時刻の線形補間との「差」としてのみ集計する。
    summary={}
    series={}
    for name in ('local','global','vio'):
        rows=trajectories[name]
        ts=np.array([r['t'] for r in rows])
        matched=[p for p in gnss if ts[0]<=p['t']<=ts[-1]]
        if matched:
            # 長い欠測をまたぐ補間は実在しない軌跡を作るため、1秒超は比較から除外。
            idx=np.searchsorted(ts,[p['t'] for p in matched],side='left')
            idx=np.clip(idx,1,len(ts)-1)
            matched=[p for p,j in zip(matched,idx) if ts[j]-ts[j-1] <= 1.0]
        if matched:
            tx=np.interp([p['t'] for p in matched],ts,[r['x'] for r in rows])
            ty=np.interp([p['t'] for p in matched],ts,[r['y'] for r in rows])
            err=np.hypot(tx-np.array([p['x'] for p in matched]),
                         ty-np.array([p['y'] for p in matched]))
            summary[name]={'samples':len(rows),'matched_gnss':len(matched),
                           'rmse_m':float(np.sqrt(np.mean(err**2))),
                           'median_m':float(np.median(err)), 'p95_m':float(np.percentile(err,95)),
                           'final_difference_m':float(err[-1])}
            series[name]=([p['t']-base['t'] for p in matched],err)
    if series:
        fig,ax=plt.subplots(figsize=(10,4))
        for key,(tt,err) in series.items():
            ax.plot(tt,err,color=colors[key],label=labels[key],linewidth=1)
        ax.set_yscale('symlog',linthresh=1.)
        ax.set_xlabel('time from initialization [s]')
        ax.set_ylabel('distance from GNSS [m]')
        ax.set_title('GNSS difference (GNSS is not ground truth)')
        ax.grid(alpha=.3);ax.legend(fontsize=8);fig.tight_layout()
        fig.savefig(args.replay_dir/'difference_vs_gnss.png',dpi=170);plt.close(fig)
    metadata={'initialization':ready,'datum_nearest_gated_fix':origin,
              'origin_note':'READY直前のgated GNSSを初期datumの近似として使用。アンテナとbaseの差は未補正。',
              'alignment':'local/globalは平行移動のみ。VIOはREADY時のlocal yawに初期角度を一度合わせる。',
              'vio_initial_rotation_deg':math.degrees(angle),
              'vio_source':'記録済み/vio/odometry。OpenVINSは再実行していない。',
              'background':'既存の同一地点OSMキャッシュ',
              'geodesy':'WGS84 Geodによる局所東北距離。全距離単位m。',
              'gate_diagnostics':{
                  topic:{'state_counts':dict(Counter(r['state'] for r in data[topic])),
                         'common_reasons':Counter(r['message'] for r in data[topic]).most_common(8)}
                  for topic in ('/vio/twist_gate/diagnostics','/vio/vertical_gate/diagnostics',
                                '/gnss/fix_gate/diagnostics')},
              'comparison':summary}
    (args.replay_dir/'plot_summary.json').write_text(
        json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(metadata,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
