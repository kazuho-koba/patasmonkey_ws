#!/usr/bin/env python3
"""同じlocal EKFへWit補正あり／なしを入力するオフラインA/B試験。

既存source comparisonのMCAP replayとCSV形式を再利用する。実機へは接続しない。
GNSS整列は行わず、元のwheel/VIO速度・TFを保持してorientationだけを変える。
"""
import argparse
import copy
import math
from pathlib import Path
import json
import numpy as np
import yaml
from mcap.reader import make_reader
from mcap.writer import Writer
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pm_evaluation.cli import compare_odometry_sources as common


def main():
    """比較用入力を生成し、同時replayの結果を時刻整列して可視化する。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--rate',type=float,default=1.0)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    derived=args.output/'inputs';derived.mkdir()
    selected={'/wit/imu','/wheel/odometry','/vio/odometry','/tf_static'}
    channels={}; schemas={}; imu_rows=[]
    with (derived/'inputs.mcap').open('wb') as f:
        writer=Writer(f);writer.start(profile='ros2')
        for path in common.find_mcap_files(args.bag):
            with path.open('rb') as source:
                for schema,channel,msg in make_reader(source).iter_messages(topics=selected):
                    if schema.name not in schemas:
                        schemas[schema.name]=writer.register_schema(name=schema.name,encoding=schema.encoding,data=schema.data)
                    outputs=[(channel.topic,msg.data)]
                    if channel.topic=='/wit/imu':
                        a=deserialize_message(msg.data,Imu);b=copy.deepcopy(a)
                        q=a.orientation;c=math.sqrt(.5)
                        # 世界z軸+90度の左乗算はdriverのEuler yaw−90度を取り消す。
                        b.orientation.x=c*(q.x-q.y);b.orientation.y=c*(q.x+q.y)
                        b.orientation.z=c*(q.z+q.w);b.orientation.w=c*(q.w-q.z)
                        outputs=[('/evaluation/imu_a',msg.data),('/evaluation/imu_b',serialize_message(b))]
                        imu_rows.append([common.stamp(a),common.yaw(a.orientation),common.yaw(b.orientation),a.angular_velocity.z])
                    for topic,data in outputs:
                        if topic not in channels:
                            channels[topic]=writer.register_channel(topic=topic,message_encoding=channel.message_encoding,schema_id=schemas[schema.name])
                        writer.add_message(channel_id=channels[topic],log_time=msg.log_time,publish_time=msg.publish_time,data=data)
        writer.finish()
    common.save_csv(args.output/'imu.csv',imu_rows,['t','yaw_a','yaw_b','wz'])
    base=yaml.safe_load(args.config.read_text())['ekf_local_node']['ros__parameters']
    configs=args.output/'configs';configs.mkdir()
    for key in ['a','b']:
        params=copy.deepcopy(base)
        # TFの競合だけ抑制する。観測マスク・共分散・モデル・frequencyは両条件共通。
        params.update(use_sim_time=True,publish_tf=False,imu0='/evaluation/imu_'+key)
        (configs/('source_'+key+'.yaml')).write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        common.SOURCE_INPUTS[key]={'/evaluation/imu_'+key,'/wheel/odometry','/vio/odometry'}
    common.replay(derived,args.output,configs,args.rate,['a','b'])
    a=common.load(args.output,'a');b=common.load(args.output,'b');imu=np.array(imu_rows)
    # 同じ開始位置へ平行移動のみ。yaw回転・GNSS fit・scale補正はしない。
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for data,label in [(a,'A: recorded -90 deg'),(b,'B: without -90 deg')]:
        t=data[:,0]-imu[0,0];yaw=np.unwrap(data[:,4])
        # Bは±180度境界を跨ぐため、A+90度に近いunwrap枝へ揃える。
        if data is b:
            yaw+=round((a[0,4]+math.pi/2-yaw[0])/(2*math.pi))*2*math.pi
        axes[0,0].plot(data[:,1]-data[0,1],data[:,2]-data[0,2],label=label)
        axes[0,1].plot(t,np.degrees(yaw),label=label)
        axes[1,0].plot(t,np.degrees(yaw-yaw[0]),label=label)
    axes[1,1].plot(imu[:,0]-imu[0,0],np.degrees(np.unwrap(imu[:,1])),label='IMU A')
    # 同じunwrap枝へ揃え、360度の見かけの差を排除する。
    axes[1,1].plot(imu[:,0]-imu[0,0],np.degrees(np.unwrap(imu[:,1])+math.pi/2),label='IMU B')
    for ax,title in zip(axes.flat,['Horizontal trajectory [m]','EKF yaw [deg]','EKF yaw change [deg]','Input IMU yaw [deg]']):
        ax.set_title(title);ax.legend();ax.grid()
    axes[0,0].set_aspect('equal',adjustable='datalim')
    for ax in [axes[0,1],axes[1,0],axes[1,1]]:ax.set_xlabel('Time [s]')
    fig.tight_layout();fig.savefig(args.output/'comparison.png')
    shared=a[(a[:,0]>=b[0,0])&(a[:,0]<=b[-1,0])]
    delta=(np.interp(shared[:,0],b[:,0],np.unwrap(b[:,4]))-np.unwrap(shared[:,4])+math.pi)%(2*math.pi)-math.pi
    summary={'bag':str(args.bag),'rate':args.rate,'counts':{'imu':len(imu),'a':len(a),'b':len(b)},
             'yaw_difference_deg_percentiles':np.degrees(np.percentile(delta,[0,50,100])).tolist(),
             'final_a_xy':a[-1,1:3].tolist(),'final_b_xy':b[-1,1:3].tolist(),
             'final_yaw_change_a_deg':float(np.degrees(np.unwrap(a[:,4])[-1]-a[0,4])),
             'final_yaw_change_b_deg':float(np.degrees(np.unwrap(b[:,4])[-1]-b[0,4]))}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2));print(summary)


if __name__=='__main__':main()
