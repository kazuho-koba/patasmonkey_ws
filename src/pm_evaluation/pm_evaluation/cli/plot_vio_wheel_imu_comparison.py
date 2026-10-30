"""記録VIO、車輪＋IMUの再計算EKF、記録localとGNSSを比較する。"""
import argparse
import bisect
import csv
from collections import Counter
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from mcap_ros2.reader import read_ros2_messages
from pyproj import Geod, Transformer
from pm_evaluation.cli.plot_recorded_local_global import read_recorded, nearest
from pm_evaluation.cli.plot_gnss_bags import read_inputs, nearest_state
from pm_evaluation.cli.plot_bag_trajectories import find_mcap_files, GNSS_STATE_STYLE
from pm_localization.vio_odom_adapter_node import quat_to_rot, openvins_odom_pose_to_matrix


def timestamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def read_reference_and_vio(bag):
    """センサ姿勢とstatic TFの経路を保持し、画像データは展開しない。"""
    local, vio, edges, gate = [], [], {}, Counter()
    for path in find_mcap_files(bag):
        for item in read_ros2_messages(path, topics=['/odometry/local','/ov_msckf/odomimu','/tf_static','/vio/twist_gate/diagnostics'], log_time_order=True):
            msg = item.ros_msg
            if item.channel.topic == '/odometry/local':
                q=msg.pose.pose.orientation
                r=quat_to_rot(q)
                local.append((timestamp(msg),math.atan2(r[1,0],r[0,0])))
            elif item.channel.topic == '/ov_msckf/odomimu':
                if msg.header.frame_id != 'global' or msg.child_frame_id != 'imu':
                    raise ValueError('OpenVINS frameが想定と異なる')
                vio.append((timestamp(msg),openvins_odom_pose_to_matrix(msg, True)))
            elif item.channel.topic == '/vio/twist_gate/diagnostics':
                for status in msg.status:
                    vals = {v.key:v.value for v in status.values}
                    gate[vals.get('state', status.message)] += 1
            else:
                for tf in msg.transforms:
                    p,q=tf.transform.translation,tf.transform.rotation
                    t=np.eye(4); t[:3,:3]=quat_to_rot(q); t[:3,3]=[p.x,p.y,p.z]
                    edges[(tf.header.frame_id,tf.child_frame_id)]=t
    local.sort(key=lambda x:x[0]); vio.sort(key=lambda x:x[0])
    return local,vio,edges,gate


def static_transform(edges, source, target):
    """TFのparent→child行列を連結し、T_source_targetを得る。"""
    queue=[(source,np.eye(4))]; seen=set()
    while queue:
        frame,mat=queue.pop(0)
        if frame==target: return mat
        if frame in seen: continue
        seen.add(frame)
        for (parent,child),tf in edges.items():
            if parent==frame and child not in seen: queue.append((child,mat@tf))
            if child==frame and parent not in seen: queue.append((parent,mat@np.linalg.inv(tf)))
    raise ValueError('static TFがありません: {} -> {}'.format(source,target))


def read_ekf(path):
    rows=[]
    for f in find_mcap_files(path):
        for item in read_ros2_messages(f, topics=['/comparison/wheel_imu'], log_time_order=True):
            msg=item.ros_msg; p=msg.pose.pose.position
            q=msg.pose.pose.orientation; r=quat_to_rot(q)
            rows.append((timestamp(msg), p.x,p.y,p.z,math.atan2(r[1,0],r[0,0])))
    rows.sort(key=lambda x:x[0]); return rows


def interpolate(rows,t):
    times=[r[0] for r in rows]; i=bisect.bisect_left(times,t)
    if i < len(rows) and abs(rows[i][0]-t) < 1e-6: return np.array(rows[i][1:])
    if i==0 or i==len(rows): return None
    a,b=rows[i-1],rows[i]
    if b[0]-a[0]>1.: return None
    w=(t-a[0])/(b[0]-a[0]); return np.array(a[1:])*(1-w)+np.array(b[1:])*w


def compare(bag, replay, basemap, summary, no_heading_offset=False, remove_magnetic_offset=False, output_dir=None):
    # 元bagと事前作成したlocal CSV・解析結果の保存先は別に指定できる。
    output_dir=bag if output_dir is None else output_dir
    output_dir.mkdir(parents=True,exist_ok=True)
    recorded=read_recorded(bag); fix=recorded['fix']; ref=summary['reference_time']
    tf=nearest(recorded['tf'],ref); locyaw,rawvio,edges,gate=read_reference_and_vio(bag)
    if not locyaw or not rawvio: raise ValueError('local方位またはOpenVINS出力なし')
    params=json.loads((bag/'provenance/effective_parameters.json').read_text())['snapshots']['/vio_odom_adapter_node']['values']
    if not params['invert_openvins_orientation']: raise ValueError('記録時のorientation設定が想定外')
    base_imu=static_transform(edges,params['base_frame_id'],params['oak_imu_frame_id'])
    local=np.loadtxt(output_dir/'recorded_local_enu.csv',delimiter=',',skiprows=1)
    local_rows=[tuple(x) for x in np.atleast_2d(local)]
    geod=Geod(ellps='WGS84'); origin=fix[0]
    def enu(lon,lat):
        az,_,d=geod.inv(origin[1],origin[2],lon,lat)
        return [d*math.sin(math.radians(az)),d*math.cos(math.radians(az))]
    _,states=read_inputs(bag)
    gnss=[(t,*enu(lon,lat),nearest_state(states,log)) for t,lon,lat,log in fix if t>=ref]
    mapyaw=math.atan2(tf[2][1,0],tf[2][0,0])
    if no_heading_offset:
        # 前回のlocalはmap→odom回転を適用済み。先頭位置を固定したまま
        # 逆回転し、odomのx/yをそのまま地図の東/北に重ねる比較条件とする。
        pivot=np.array(local_rows[0][1:4]); inv=rotation(-mapyaw)
        local_rows=[(v[0],*(pivot+inv@(np.array(v[1:4])-pivot))) for v in local_rows]
    # 出力の最初の共通時刻で記録localの位置とyawを合わせる。平面EKFは
    # yawの一定回転に対して等価なので、0 rad起動の再計算を後から回転できる。
    ekf=read_ekf(replay/'replayed_bag'); start=max(ref,ekf[0][0],local_rows[0][0]); ekf=[r for r in ekf if r[0]>=start]
    start=ekf[0][0]; anchor=interpolate(local_rows,start); heading=nearest(locyaw,start)
    if anchor is None or abs(heading[0]-start)>.2: raise ValueError('EKF開始時の位置・方位を対応できない')
    R=np.eye(3) if no_heading_offset else rotation(heading[1]+mapyaw-ekf[0][4]); p0=np.array(ekf[0][1:4])
    wheel=[(v[0],*(anchor+R@(np.array(v[1:4])-p0))) for v in ekf]
    # OpenVINSのglobal→IMU poseを取り付けTFでbase_linkへ変換し、最初の
    # 観測姿勢のyawと記録localの同時刻yawを合わせる。GNSS回転fitは行わない。
    rawvio=[v for v in rawvio if v[0]>=ref and v[0]>=local_rows[0][0]]
    t0=rawvio[0][0]; anchor_v=interpolate(local_rows,t0); h=nearest(locyaw,t0)
    if anchor_v is None or abs(h[0]-t0)>.2: raise ValueError('VIO開始時の位置・方位を対応できない')
    imu_base=np.linalg.inv(base_imu); first=rawvio[0][1]@imu_base
    first_yaw=math.atan2(first[1,0],first[0,0]); rotate=np.eye(3) if no_heading_offset else rotation(h[1]+mapyaw-first_yaw)
    vo=[]
    for t,pose in rawvio:
        p=(pose@imu_base)[:3,3]; v=anchor_v+rotate@(p-first[:3,3]); vo.append((t,*v))
    series=[('GNSS',gnss,'black'),('Recorded local',local_rows,'tab:blue'),('Wheel vx + Wit wz EKF',wheel,'tab:orange'),('OpenVINS pose',vo,'tab:purple')]
    if remove_magnetic_offset:
        # bearing=atan2(mag)+offset、yaw=pi/2-bearingであるため、
        # 保存されたoffsetを0にするとyawの変化量は元のoffsetと同じ符号になる。
        # bag以前の初期位置・ゲート状態は不明なのでbag基準点での近似回転。
        snapshot=json.loads((bag/'provenance/effective_parameters.json').read_text())
        original_offset=float(snapshot['snapshots']['/heading_initializer_node']['values']['offset_deg'])
        pivot=np.array(local_rows[0][1:4]); adjustment=rotation(math.radians(original_offset))
        series=[(name,rows if name=='GNSS' else [(v[0],*(pivot+adjustment@(np.array(v[1:4])-pivot))) for v in rows],color) for name,rows,color in series]
        local_rows,wheel,vo=[series[i][1] for i in (1,2,3)]
    def metric(rows, since=-math.inf, until=math.inf):
        values=[]
        for g in gnss:
            if not since <= g[0] <= until: continue
            p=interpolate(rows,g[0])
            if p is not None: values.append(math.hypot(p[0]-g[1],p[1]-g[2]))
        return {'count':len(values),'median_m':float(np.median(values)) if values else None,'p95_m':float(np.percentile(values,95)) if values else None}
    common_start=max(rows[0][0] for _,rows,_ in series); common_end=min(rows[-1][0] for _,rows,_ in series)
    result={'magnetic_offset_removed_approx':remove_magnetic_offset,'magnetic_offset_original_deg':original_offset if remove_magnetic_offset else None,'heading_offset_applied':not no_heading_offset,'map_odom_yaw_deg':math.degrees(mapyaw),'vio_alignment_rotation_deg':0.0 if no_heading_offset else math.degrees(h[1]+mapyaw-first_yaw),'wheel_alignment_rotation_deg':0.0 if no_heading_offset else math.degrees(heading[1]+mapyaw-ekf[0][4]),'common_start':common_start,'common_end':common_end,'common_metrics':{name:metric(rows,common_start,common_end) for name,rows,_ in series[1:]},'bag':bag.name,'initial_reference_time':ref,'wheel_alignment_time':start,'wheel_yaw_enu_deg':math.degrees(heading[1]+mapyaw),'vio_alignment_time':t0,'vio_yaw_enu_deg':math.degrees(h[1]+mapyaw),'vio_initial_lag_s':t0-ref,'base_imu_frame':params['oak_imu_frame_id'],'metrics':{name:metric(rows) for name,rows,_ in series[1:]},'counts':{name:len(rows) for name,rows,_ in series},'vio_gate_diagnostic_counts':dict(gate)}
    suffix='_magnetic_offset_zero_approx' if remove_magnetic_offset else ('_no_heading_offset' if no_heading_offset else '')
    for name,rows,_ in series[1:]:
        with (output_dir/('source_comparison_'+{'Recorded local':'local','Wheel vx + Wit wz EKF':'wheel_imu','OpenVINS pose':'vio'}[name]+suffix+'.csv')).open('w') as f:
            w=csv.writer(f);w.writerow(['stamp','east_m','north_m','z_m']);w.writerows(rows)
    bg=np.load(basemap); ext=bg['extent']; tr=Transformer.from_crs('EPSG:3857','EPSG:4326',always_xy=True); lo=enu(*tr.transform(ext[0],ext[2])); hi=enu(*tr.transform(ext[1],ext[3]))
    fig,ax=plt.subplots(figsize=(11,9)); ax.imshow(bg['image'],extent=[lo[0],hi[0],lo[1],hi[1]],origin='upper')
    ax.plot([r[1] for r in gnss],[r[2] for r in gnss],color='black',alpha=.4,lw=1,label='GNSS /fix')
    for state,style in GNSS_STATE_STYLE.items():
        sub=[r for r in gnss if r[3]==state]
        if sub: ax.scatter([r[1] for r in sub],[r[2] for r in sub],s=12,c=style['color'],marker=style['marker'],label=style['label'])
    for name,rows,color in series[1:]: ax.plot([r[1] for r in rows],[r[2] for r in rows],color=color,lw=1.3,label=name)
    ax.scatter([gnss[0][1]],[gnss[0][2]],marker='^',s=110,c='limegreen',edgecolors='black',label='GNSS start',zorder=9)
    ax.scatter([gnss[-1][1]],[gnss[-1][2]],marker='X',s=110,c='red',edgecolors='black',label='GNSS goal',zorder=9)
    ax.scatter([vo[0][1]],[vo[0][2]],marker='o',s=50,c='purple',edgecolors='white',label='VIO start',zorder=9)
    ax.set(xlabel='East [m]',ylabel='North [m]',title=bag.name+' — source comparison'+(' (magnetic offset zero, approx.)' if remove_magnetic_offset else (' (no heading offset)' if no_heading_offset else ''))); ax.set_aspect('equal'); ax.grid(alpha=.25); ax.legend(fontsize=8)
    ax.text(.99,.01,'Map: © OpenStreetMap contributors',transform=ax.transAxes,ha='right',fontsize=8,bbox={'facecolor':'white','alpha':.8,'edgecolor':'none'})
    xs=[v[1] for v in gnss];ys=[v[2] for v in gnss]; margin=max(10.,.1*max(max(xs)-min(xs),max(ys)-min(ys)))
    ax.set_xlim(min(xs)-margin,max(xs)+margin);ax.set_ylim(min(ys)-margin,max(ys)+margin);fig.tight_layout();fig.savefig(output_dir/('source_comparison_gnss_osm'+suffix+'.png'),dpi=170)
    for _,rows,_ in series[1:]: xs.extend(v[1] for v in rows);ys.extend(v[2] for v in rows)
    ax.set_xlim(min(xs)-margin,max(xs)+margin);ax.set_ylim(min(ys)-margin,max(ys)+margin);fig.savefig(output_dir/('source_comparison_gnss_osm_full'+suffix+'.png'),dpi=170);plt.close(fig)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag_root',type=Path);p.add_argument('--replay-root',type=Path,required=True);p.add_argument('--basemap',type=Path,required=True);p.add_argument('--reference-summary',type=Path,required=True);p.add_argument('--output-root',type=Path,help='入力local CSVと結果の保存先');p.add_argument('--no-heading-offset',action='store_true',help='方位合わせの回転を適用せず各ソースのx/yを地図上に重ねる');p.add_argument('--remove-magnetic-offset',action='store_true',help='記録した磁気offsetをゼロにした場合の概算軌跡'); args=p.parse_args()
    if args.no_heading_offset and args.remove_magnetic_offset: p.error('両オプションは同時に指定できません')
    summaries={x['bag']:x for x in json.loads(args.reference_summary.read_text())};results=[]
    for bag in sorted(args.bag_root.glob('rosbag2_2026_10_30-15_*')):
        try:
            r=compare(bag,args.replay_root/bag.name,args.basemap,summaries[bag.name],args.no_heading_offset,args.remove_magnetic_offset,args.output_root/bag.name if args.output_root else None);r['status']='ok'
        except Exception as exc:
            r={'bag':bag.name,'status':'error','reason':repr(exc)}
        print(json.dumps(r,ensure_ascii=False),flush=True);results.append(r)
    (args.replay_root/('summary_magnetic_offset_zero_approx.json' if args.remove_magnetic_offset else ('summary_no_heading_offset.json' if args.no_heading_offset else 'summary.json'))).write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    if any(r['status']!='ok' for r in results): raise SystemExit(1)

if __name__=='__main__': main()
