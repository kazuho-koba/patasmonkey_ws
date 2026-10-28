"""global EKF発散とGNSS・車輪入力の時刻関係を調べる。"""
import argparse
import csv
import json
import math
from pathlib import Path
from collections import Counter

from mcap.reader import make_reader
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay_dir',type=Path)
    args=parser.parse_args()
    topics={'/odometry/global','/odometry/local','/odometry/gps','/wheel/odometry',
            '/vio/odometry/twist_gated'}
    rows={key:[] for key in topics}
    for path in sorted((args.replay_dir/'replayed_bag').glob('*.mcap')):
        with path.open('rb') as stream:
            for _,channel,record in make_reader(stream).iter_messages(topics=list(topics)):
                m=deserialize_message(record.data,Odometry)
                p=m.pose.pose.position
                v=m.twist.twist.linear
                w=m.twist.twist.angular
                q=m.pose.pose.orientation
                rows[channel.topic].append(dict(
                    t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                    wall=record.log_time*1e-9,x=p.x,y=p.y,z=p.z,
                    vx=v.x,vy=v.y,vz=v.z,wz=w.z,
                    qx=q.x,qy=q.y,qz=q.z,qw=q.w,
                    cov_x=m.pose.covariance[0],cov_y=m.pose.covariance[7],
                    cov_z=m.pose.covariance[14],cov_yaw=m.pose.covariance[35]))
    for topic in topics:
        name=topic.strip('/').replace('/','_')+'_audit.csv'
        if rows[topic]:
            with (args.replay_dir/name).open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(rows[topic][0]))
                writer.writeheader();writer.writerows(rows[topic])
    status=json.loads((args.replay_dir/'plot_summary.json').read_text())
    ready=status['initialization']['wall']
    glob=[r for r in rows['/odometry/global'] if r['wall']>=ready]
    gps=[r for r in rows['/odometry/gps'] if r['wall']>=ready]
    wheel=rows['/wheel/odometry']
    start=glob[0]['t']
    result={'ready_global_header_sec':start,'gated_vio_count':len(rows['/vio/odometry/twist_gated']),
            'global_crossings':{},'gps_max_abs':{},
            'wheel_speed_over_2p5_count':sum(abs(r['vx'])>2.5 for r in wheel)}
    for limit in (10,50,100,1000,10000):
        row=next((r for r in glob if math.hypot(r['x'],r['y'])>limit),None)
        if row:
            result['global_crossings'][str(limit)]={'after_ready_s':row['t']-start,**row}
    for key in ('x','y','z'):
        result['gps_max_abs'][key]=max((abs(r[key]) for r in gps),default=None)
    result['global_max_speed_mps']=max(math.sqrt(r['vx']**2+r['vy']**2+r['vz']**2) for r in glob)
    result['global_max_abs_altitude_m']=max(abs(r['z']) for r in glob)
    result['extreme_wheel']=[{'after_ready_s':r['t']-start,**r} for r in wheel if abs(r['vx'])>2.5][:20]
    (args.replay_dir/'global_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
