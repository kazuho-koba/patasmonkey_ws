"""再生bagのTFとGNSS・EKF出力を対応付け、座標変換の循環を検証する。"""
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
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def rotate(q, v):
    """quaternionで3次元vectorを回転し、TFのtranslationを加える前の座標を得る。"""
    x,y,z,w=q
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-9:
        raise ValueError('無効なTF quaternion')
    qv=np.array((x,y,z))/norm
    w=w/norm
    v=np.asarray(v,dtype=float)
    return v+2*np.cross(qv,np.cross(qv,v)+w*v)


def nearest_index(times, value):
    index=int(np.searchsorted(times,value))
    if index<=0:
        return 0
    if index>=len(times):
        return len(times)-1
    return index if abs(times[index]-value)<abs(times[index-1]-value) else index-1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay_dir',type=Path)
    parser.add_argument('--max-time-difference',type=float,default=.15,
                        help='TF・local・globalのheader対応許容差 [s]')
    args=parser.parse_args()
    topics={'/tf':TFMessage,'/odometry/gps':Odometry,'/odometry/local':Odometry,
            '/odometry/global':Odometry,'/localization/initialization/status':String}
    tf=[]; gps=[]; local=[]; global_=[]; ready=None
    for path in sorted((args.replay_dir/'replayed_bag').glob('*.mcap')):
        with path.open('rb') as stream:
            for _,channel,record in make_reader(stream).iter_messages(topics=list(topics)):
                topic=channel.topic
                msg=deserialize_message(record.data,topics[topic])
                wall=record.log_time*1e-9
                if topic.endswith('/status'):
                    state=json.loads(msg.data)
                    if ready is None and state.get('state')=='READY':
                        ready=wall
                elif topic=='/tf':
                    for transform in msg.transforms:
                        if transform.header.frame_id=='map' and transform.child_frame_id=='odom':
                            v=transform.transform.translation
                            q=transform.transform.rotation
                            stamp=transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9
                            tf.append(dict(t=stamp,p=[v.x,v.y,v.z],q=[q.x,q.y,q.z,q.w]))
                else:
                    pos=msg.pose.pose.position
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                    row=dict(t=stamp,wall=wall,frame=msg.header.frame_id,p=[pos.x,pos.y,pos.z])
                    {'/odometry/gps':gps,'/odometry/local':local,
                     '/odometry/global':global_}[topic].append(row)
    if ready is None:
        raise SystemExit('初期化READYが記録されていません')
    if not tf or not gps or not local or not global_:
        raise SystemExit('TF、GPS、local、globalのいずれかが記録されていません')
    for series in (tf,gps,local,global_):
        series.sort(key=lambda r:r['t'])
    tt,lt,gt=(np.array([r['t'] for r in series]) for series in (tf,local,global_))
    gps=[r for r in gps if r['wall']>=ready]
    start=gps[0]['t']
    output=[]; skipped=0
    for measurement in gps:
        t=measurement['t']
        frame=measurement['frame']
        transform=tf[nearest_index(tt,t)]
        loc=local[nearest_index(lt,t)]
        glob=global_[nearest_index(gt,t)]
        differences=[abs(transform['t']-t),abs(loc['t']-t),abs(glob['t']-t)]
        if max(differences)>args.max_time_difference:
            skipped+=1
            continue
        if frame=='odom':
            p_map=rotate(transform['q'],measurement['p'])+np.array(transform['p'])
            residual_odom=np.array(measurement['p'])-np.array(loc['p'])
        elif frame=='map':
            p_map=np.array(measurement['p'])
            residual_odom=np.array([float('nan')]*3)
        else:
            raise ValueError('予期しないGNSS frame: {}'.format(frame))
        # 同時刻に近い各出力を使うため、これはEKF内部の事前innovationそのもの
        # ではない。変換後の入力と出力の幾何学的関係を追う監査値である。
        residual_map=p_map-np.array(glob['p'])
        composed_local=rotate(transform['q'],loc['p'])+np.array(transform['p'])
        consistency=np.linalg.norm(composed_local-np.array(glob['p']))
        row=dict(t=t,elapsed_s=t-start,frame=frame,tf_dt=differences[0],
                 local_dt=differences[1],global_dt=differences[2],
                 tf_x=transform['p'][0],tf_y=transform['p'][1],tf_z=transform['p'][2],
                 gps_input_x=measurement['p'][0],gps_input_y=measurement['p'][1],
                 gps_map_x=float(p_map[0]),gps_map_y=float(p_map[1]),
                 global_x=glob['p'][0],global_y=glob['p'][1],
                 local_x=loc['p'][0],local_y=loc['p'][1],
                 map_difference_m=float(np.linalg.norm(residual_map)),
                 odom_difference_m=float(np.linalg.norm(residual_odom)),
                 tf_global_consistency_m=float(consistency))
        output.append(row)
    if not output:
        raise SystemExit('時刻が一致するTF・GPS・EKFの組がありません')
    with (args.replay_dir/'tf_gnss_audit.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(output[0]));writer.writeheader();writer.writerows(output)
    frame=output[0]['frame']
    def stats(name):
        vals=np.array([r[name] for r in output])
        return {'median':float(np.nanmedian(vals)),'p95':float(np.nanpercentile(vals,95)),
                'max':float(np.nanmax(vals))}
    finite=lambda n: [r[n] for r in output]
    summary={'gps_frame':frame,'matches':len(output),'skipped_time_mismatch':skipped,
             'tf_match_max_s':max(finite('tf_dt')),
             'tf_global_consistency_m':stats('tf_global_consistency_m'),
             'gps_map_minus_global_m':stats('map_difference_m'),
             'first':output[0], 'last':output[-1]}
    if frame=='odom':
        summary['gps_odom_minus_local_m']=stats('odom_difference_m')
        # TFの平行移動とglobal位置が同方向へ大きく動くかを時系列で調べる。
        summary['global_vs_transformed_gps_y_correlation']=float(
            np.corrcoef(finite('global_y'),finite('gps_map_y'))[0,1])
    (args.replay_dir/'tf_gnss_audit.json').write_text(
        json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    t=finite('elapsed_s')
    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
    for key,label in [('global_y','global y in map'),
                      ('gps_map_y','GPS y transformed into map'),
                      ('gps_input_y','GPS y as published')]:
        axes[0].plot(t,finite(key),label=label,linewidth=1)
    axes[0].set_ylabel('north / y [m]');axes[0].legend()
    axes[1].plot(t,finite('map_difference_m'),label='GPS in map minus global [3D m]')
    if frame=='odom':
        axes[1].plot(t,finite('odom_difference_m'),label='GPS in odom minus local [3D m]')
    axes[1].set_xlabel('time from first GPS after READY [s]')
    axes[1].set_ylabel('distance [m]');axes[1].legend()
    for ax in axes:ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(args.replay_dir/'tf_gnss_audit.png',dpi=150);plt.close(fig)
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
