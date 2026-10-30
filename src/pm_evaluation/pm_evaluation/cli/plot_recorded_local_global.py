"""記録済みlocal/global odometryを再計算せずGNSS・地図と重ねる。"""
import argparse
import bisect
from collections import Counter
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages
import numpy as np
from pyproj import Geod, Transformer

from pm_evaluation.cli.plot_gnss_bags import read_inputs, nearest_state
from pm_evaluation.cli.plot_bag_trajectories import GNSS_STATE_STYLE, find_mcap_files
from pm_localization.vio_odom_adapter_node import quat_to_rot


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read_recorded(bag):
    topics = ['/odometry/local', '/odometry/global', '/odometry/gps', '/fix', '/tf',
              '/localization/initialization/status']
    result = {'local':[], 'global':[], 'gps':[], 'fix':[], 'tf':[], 'states':Counter()}
    for path in find_mcap_files(bag):
        for record in read_ros2_messages(path, topics=topics, log_time_order=True):
            name, msg = record.channel.topic, record.ros_msg
            if name == '/tf':
                for tf in msg.transforms:
                    if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                        p, q = tf.transform.translation, tf.transform.rotation
                        result['tf'].append((stamp(tf.header), np.array([p.x,p.y,p.z]), quat_to_rot(q)))
            elif name == '/fix':
                if msg.status.status >= 0 and math.isfinite(msg.longitude) and math.isfinite(msg.latitude):
                    result['fix'].append((stamp(msg.header), msg.longitude, msg.latitude, int(record.log_time_ns)))
            elif name.endswith('/status'):
                try: result['states'][json.loads(msg.data).get('state','UNKNOWN')] += 1
                except (ValueError, AttributeError): pass
            else:
                key = name.rsplit('/',1)[-1]
                p = msg.pose.pose.position
                if all(math.isfinite(v) for v in (p.x,p.y,p.z)):
                    result[key].append((stamp(msg.header),p.x,p.y,p.z,msg.header.frame_id,msg.child_frame_id))
    for key in ('local','global','gps','fix','tf'):
        result[key].sort(key=lambda row:row[0])
    return result


def nearest(rows, t):
    times=[row[0] for row in rows]; i=bisect.bisect_left(times,t)
    return min(rows[max(0,i-1):min(len(rows),i+1)],key=lambda row:abs(row[0]-t))


def evaluate(bag, basemap, output_dir=None):
    # 入力bagと解析結果の保存先を分け、bag本体は読み取り専用で扱える。
    output_dir=bag if output_dir is None else output_dir
    output_dir.mkdir(parents=True,exist_ok=True)
    data=read_recorded(bag)
    _, states=read_inputs(bag)
    fixes=data['fix']
    if not fixes or any(not data[key] for key in ('local','global','gps','tf')):
        raise RuntimeError('GNSS/local/global/GPS変換またはmap→odom TFがありません')
    if (any(x[4]!='odom' or x[5]!='base_link' for x in data['local']) or
        any(x[4]!='map' or x[5]!='base_link' for x in data['global']) or
        any(x[4]!='map' for x in data['gps'])):
        raise RuntimeError('odom/global/gpsのframe_idが想定と異なります')
    # 原点は最初の有効GNSS fix。位置合わせには同時刻のnavsat出力だけを使い、
    # 軌跡に対する回転fit・任意の位置合わせは実施しない。
    first=fixes[0]; geod=Geod(ellps='WGS84')
    def enu(lon,lat):
        az,_,distance=geod.inv(first[1],first[2],lon,lat)
        return np.array([distance*math.sin(math.radians(az)),distance*math.cos(math.radians(az))])
    gps=data['gps']
    matched=[]
    for seconds,lon,lat,_ in fixes:
        point=nearest(gps,seconds)
        if abs(point[0]-seconds)<=.25:
            matched.append((seconds,point,enu(lon,lat)))
    if not matched:
        raise RuntimeError('GNSSと/odometry/gpsの同時刻ペアがありません')
    ref_time=max(data['local'][0][0],data['global'][0][0],matched[0][0])
    pair=next((v for v in matched if v[0]>=ref_time),None)
    if pair is None: raise RuntimeError('基準時刻以降のGNSS対応がありません')
    ref_time=pair[0]
    offset=pair[2]-np.array(pair[1][1:3])
    tf=nearest(data['tf'],ref_time)
    if abs(tf[0]-ref_time)>.25:
        raise RuntimeError('基準時刻のmap→odom TFがありません')
    initial_local=nearest(data['local'],ref_time)
    initial_global=nearest(data['global'],ref_time)
    initial_map=tf[2] @ np.array(initial_local[1:4])+tf[1]
    initial_residual=float(np.linalg.norm(initial_map[:2]-np.array(initial_global[1:3])))
    # localは最初のTFで一度だけmapへ変換する。各時刻のTFで変換するとglobalへの
    # GNSS補正がlocalにも入るため、元のlocalのドリフトを別途評価できない。
    local=[]
    for t,x,y,z,_,_ in data['local']:
        if t>=ref_time:
            xyz=tf[2] @ np.array([x,y,z])+tf[1]
            local.append((t,xyz[0]+offset[0],xyz[1]+offset[1],z))
    global_=[(t,x+offset[0],y+offset[1],z) for t,x,y,z,_,_ in data['global'] if t>=ref_time]
    gnss=[(t,*enu(lon,lat),nearest_state(states,log)) for t,lon,lat,log in fixes if t>=ref_time]
    if not local or not global_ or not gnss: raise RuntimeError('共通時刻以降の軌跡がありません')
    comparison={}
    for name,rows in [('local',local),('global',global_)]:
        times=np.asarray([r[0] for r in rows]); xs=np.asarray([r[1] for r in rows]); ys=np.asarray([r[2] for r in rows])
        values=[]
        for t,x,y,_ in gnss:
            i=int(np.searchsorted(times,t))
            if i==0 or i==len(times) or times[i]-times[i-1] > 1.: continue
            values.append(math.hypot(float(np.interp(t,times,xs))-x,float(np.interp(t,times,ys))-y))
        comparison[name]={'count':len(values),'median_m':float(np.median(values)) if values else None,
                          'p95_m':float(np.percentile(values,95)) if values else None}
    background=np.load(basemap); extent=background['extent']
    transformer=Transformer.from_crs('EPSG:3857','EPSG:4326',always_xy=True)
    lo=enu(*transformer.transform(extent[0],extent[2])); hi=enu(*transformer.transform(extent[1],extent[3]))
    fig,ax=plt.subplots(figsize=(10,9))
    ax.imshow(background['image'],extent=[lo[0],hi[0],lo[1],hi[1]],origin='upper',zorder=0)
    ax.plot([v[1] for v in gnss],[v[2] for v in gnss],color='black',alpha=.4,lw=1,label='GNSS /fix')
    for state,style in GNSS_STATE_STYLE.items():
        selected=[v for v in gnss if v[3]==state]
        if selected: ax.scatter([v[1] for v in selected],[v[2] for v in selected],s=15,c=style['color'],marker=style['marker'],label=style['label'],zorder=3)
    for name,rows,color in [('Local (initial TF fixed)',local,'tab:blue'),('Global (map)',global_,'tab:red')]:
        ax.plot([v[1] for v in rows],[v[2] for v in rows],color=color,lw=1.3,label=name,zorder=4)
    for row,mark,color,label,offset_text in [(gnss[0],'^','limegreen','START',(10,14)),(gnss[-1],'X','crimson','GOAL',(-45,-22))]:
        ax.scatter([row[1]],[row[2]],marker=mark,s=130,c=color,edgecolors='black',zorder=8,label=label)
        ax.annotate(label,(row[1],row[2]),xytext=offset_text,textcoords='offset points',color=color,
                    fontweight='bold',arrowprops={'arrowstyle':'->','color':color})
    ax.set_aspect('equal',adjustable='box'); ax.grid(alpha=.25); ax.legend(fontsize=8)
    ax.set(xlabel='East [m]',ylabel='North [m]',title=bag.name+' — recorded odometry')
    ax.text(.99,.01,'Map: © OpenStreetMap contributors',transform=ax.transAxes,ha='right',fontsize=8,
            bbox={'facecolor':'white','alpha':.8,'edgecolor':'none'})
    gx=[r[1] for r in gnss]; gy=[r[2] for r in gnss]
    margin=max(10., .1*max(max(gx)-min(gx),max(gy)-min(gy)))
    ax.set_xlim(min(gx)-margin,max(gx)+margin); ax.set_ylim(min(gy)-margin,max(gy)+margin)
    fig.tight_layout(); fig.savefig(output_dir/'recorded_odom_gnss_osm.png',dpi=170)
    allx=gx+[r[1] for r in local]+[r[1] for r in global_]; ally=gy+[r[2] for r in local]+[r[2] for r in global_]
    ax.set_xlim(min(allx)-margin,max(allx)+margin); ax.set_ylim(min(ally)-margin,max(ally)+margin)
    ax.set_title(bag.name+' — recorded odometry (full extent)')
    fig.savefig(output_dir/'recorded_odom_gnss_osm_full.png',dpi=170); plt.close(fig)
    for name,rows in [('local',local),('global',global_)]:
        with (output_dir/('recorded_'+name+'_enu.csv')).open('w',newline='') as f:
            w=csv.writer(f); w.writerow(['stamp','east_m','north_m','z_m']); w.writerows(rows)
    return {'bag':bag.name, 'reference_time':ref_time,'tf_time_difference_s':tf[0]-ref_time,
            'gps_fix_time_difference_s':pair[1][0]-pair[0], 'status_counts':dict(data['states']),
            'initial_tf_local_global_difference_m':initial_residual,
            'fix_count':len(gnss),'local_count':len(local),'global_count':len(global_),
            'gnss_distance':comparison,'map_odom_yaw_deg':math.degrees(math.atan2(tf[2][1,0],tf[2][0,0]))}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags',nargs='+',type=Path)
    parser.add_argument('--basemap',type=Path,required=True)
    parser.add_argument('--summary',type=Path,required=True)
    parser.add_argument('--output-root',type=Path,help='解析結果の保存先。bag名のディレクトリを作成する')
    args=parser.parse_args()
    results=[]
    for bag in args.bags:
        try:
            result=evaluate(bag,args.basemap,args.output_root/bag.name if args.output_root else None); result['status']='ok'
        except Exception as error:
            result={'bag':bag.name,'status':'error','reason':str(error)}
        results.append(result); print(json.dumps(result,ensure_ascii=False),flush=True)
    args.summary.write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

if __name__ == '__main__':
    main()
