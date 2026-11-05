"""OpenVINS adapterの剛体変換・共分散をbagで監査する。ROS nodeは起動しない。"""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from mcap_ros2.reader import read_ros2_messages
from pm_evaluation.cli.plot_bag_trajectories import find_mcap_files
from pm_evaluation.cli.plot_vio_wheel_imu_comparison import static_transform, rotation
from pm_localization.vio_odom_adapter_node import (
    quat_to_rot, rot_to_quat, so3_log, transform_to_matrix,
    openvins_odom_pose_to_matrix, calculate_corrected_output_rotation,
    transform_pose_covariance_ros_approx, correct_pose_covariance_for_output_rotation,
    transform_twist_and_covariance_ros_approx, correct_twist_velocity,
    correct_twist_covariance, twist_axis_fix_matrix3,
)
from pm_localization.vio_rigid_transform import transform_pose, transform_twist


def key(msg):
    return msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec


def summary(values):
    a=np.asarray(values,dtype=float)
    return {'count':len(a),'median':float(np.median(a)) if len(a) else None,
            'p95':float(np.percentile(a,95)) if len(a) else None,
            'max':float(np.max(a)) if len(a) else None}


def analyze(bag, output, sample_hz):
    """旧方式は記録設定、候補はROS姿勢＋仮取付回転で同じrawを変換する。

    初期配置を一度だけ固定する。候補はyawだけをゼロにして重力方向を維持。
    実機adapterの初期化がbag開始前なら旧方式の初期姿勢は復元不能なので、
    actual一致はそのまま正常性の判定に使わず、基準の不足を出力に記載する。
    """
    output.mkdir(parents=True,exist_ok=True)
    raw, actual, edges = {}, {}, {}
    for path in find_mcap_files(bag):
        for item in read_ros2_messages(path,topics=['/ov_msckf/odomimu','/vio/odometry','/tf_static'],log_time_order=True):
            msg=item.ros_msg
            if item.channel.topic=='/tf_static':
                for tf in msg.transforms:
                    edges.setdefault((tf.header.frame_id,tf.child_frame_id),transform_to_matrix(tf))
            elif item.channel.topic=='/ov_msckf/odomimu': raw[key(msg)]=msg
            else:actual[key(msg)]=msg
    if not raw: raise ValueError('OpenVINSデータなし')
    config_path=bag/'provenance/effective_parameters.json'
    params={'base_frame_id':'base_link','oak_imu_frame_id':'openvins_imu_link',
            'invert_openvins_orientation':True,'align_initial_to_tf':False,'zero_initial_pose':True,
            'align_output_orientation_to_initial_tf':True,'invert_relative_rotation':True}
    if config_path.exists():
        params.update(json.loads(config_path.read_text())['snapshots']['/vio_odom_adapter_node']['values'])
    if params['align_initial_to_tf'] or not params['zero_initial_pose']:
        raise ValueError('本監査CLIはTF整列なし・初回位置ゼロ化の記録設定を対象とする')
    base_tf=static_transform(edges,params['base_frame_id'],params['oak_imu_frame_id'])
    old_inverse=np.linalg.inv(base_tf)
    c=twist_axis_fix_matrix3()@base_tf[:3,:3];lever=base_tf[:3,3]
    common=sorted(set(raw).intersection(actual))
    first_key=common[0] if common else min(raw)
    first=raw[first_key]
    old_first=openvins_odom_pose_to_matrix(first,params['invert_openvins_orientation'])@old_inverse
    native_first=openvins_odom_pose_to_matrix(first,False)
    physical_first=native_first[:3,:3]@c.T
    yaw=math.atan2(physical_first[1,0],physical_first[0,0]);a=rotation(-yaw)
    native_origin=native_first[:3,3]-native_first[:3,:3]@c.T@lever
    offset=-a@native_origin
    table=[];old_p=[];old_r=[];old_v=[];old_w=[];new_p=[];new_r=[];new_v=[];new_w=[];times=[]
    covariance_differences=[];old_actual_p=[];old_actual_v=[];old_actual_cov=[];invalid=[];min_eigen=[]
    chosen=[];last=-math.inf
    for k in sorted(raw):
        if k<first_key:continue
        t=k*1e-9
        if t-last<1/sample_hz:continue
        chosen.append(k);last=t
    for k in chosen:
        m=raw[k];time=k*1e-9
        if m.header.frame_id!='global' or m.child_frame_id!='imu':raise ValueError('予期しないraw frame')
        q=m.pose.pose.orientation
        norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if not math.isfinite(norm) or norm<1e-12:
            invalid.append({'stamp':time,'reason':'無効なquaternion'});continue
        native=openvins_odom_pose_to_matrix(m,False)
        old_T=openvins_odom_pose_to_matrix(m,params['invert_openvins_orientation'])@old_inverse
        po=old_T[:3,3]-old_first[:3,3]
        ro=calculate_corrected_output_rotation(old_T[:3,:3],old_first[:3,:3],None,
            params['align_output_orientation_to_initial_tf'],params['invert_relative_rotation'])
        pco=transform_pose_covariance_ros_approx(m.pose.covariance,np.eye(3),
            openvins_odom_pose_to_matrix(m,params['invert_openvins_orientation'])[:3,:3],old_inverse[:3,3])
        pco=np.asarray(correct_pose_covariance_for_output_rotation(pco,old_T[:3,:3],old_first[:3,:3],None,
            params['align_output_orientation_to_initial_tf'],params['invert_relative_rotation'],orientation_var_floor=.05)).reshape(6,6)
        vo,wo,tco=transform_twist_and_covariance_ros_approx(m.twist.twist,m.twist.covariance,base_tf[:3,:3],lever)
        vo,wo=correct_twist_velocity(vo,wo);tco=np.asarray(correct_twist_covariance(tco)).reshape(6,6)
        linear=m.twist.twist.linear;angular=m.twist.twist.angular
        vi=np.array([linear.x,linear.y,linear.z,angular.x,angular.y,angular.z])
        try:
            pn,rn,pcn,_=transform_pose(native[:3,3],native[:3,:3],m.pose.covariance,c,lever,a,offset)
            vn,tcn,_=transform_twist(vi,m.twist.covariance,c,lever)
        except ValueError as exc:
            invalid.append({'stamp':time,'reason':str(exc)});continue
        times.append(time);old_p.append(po);old_r.append(ro);old_v.append(vo);old_w.append(wo)
        new_p.append(pn);new_r.append(rn);new_v.append(vn[:3]);new_w.append(vn[3:])
        covariance_differences.append(np.linalg.norm(tco-tcn))
        min_eigen.append(min(np.linalg.eigvalsh(pcn).min(),np.linalg.eigvalsh(tcn).min()))
        if k in actual:
            recorded=actual[k];p=recorded.pose.pose.position;v=recorded.twist.twist.linear
            old_actual_p.append(np.linalg.norm(po-[p.x,p.y,p.z]));old_actual_v.append(np.linalg.norm(vo-[v.x,v.y,v.z]))
            old_actual_cov.append(max(np.max(np.abs(pco-np.asarray(recorded.pose.covariance).reshape(6,6))),np.max(np.abs(tco-np.asarray(recorded.twist.covariance).reshape(6,6)))))
        table.append([time,*po,*pn,*vo,*vn[:3],*np.diag(pco),*np.diag(pcn),*np.diag(tco),*np.diag(tcn)])
    if len(times)<3:raise ValueError('有効サンプル不足')
    times=np.asarray(times);old_p=np.asarray(old_p);new_p=np.asarray(new_p);old_v=np.asarray(old_v);new_v=np.asarray(new_v)
    def residual(positions,rotations,velocities,angular):
        linear=[];rotation_error=[]
        for i in range(1,len(times)-1):
            dt=times[i+1]-times[i-1]
            if dt<=0 or dt>4/sample_hz:continue
            derivative=(positions[i+1]-positions[i-1])/dt
            linear.append(np.linalg.norm(derivative-rotations[i]@velocities[i]))
            gyro=so3_log(rotations[i-1].T@rotations[i+1])/dt
            rotation_error.append(np.linalg.norm(gyro-angular[i]))
        return {'linear_mps':summary(linear),'angular_rad_s':summary(rotation_error)}
    velocity_difference=np.linalg.norm(old_v-new_v,axis=1)
    result={'bag':str(bag),'status':'ok','raw_count':len(raw),'adapter_count':len(actual),
        'matched_headers':len(common),'sample_count':len(times),'sample_hz':sample_hz,
        'initial_reference':'最初の共通header、actualがなければ最初のraw。bag前の状態は未復元',
        'candidate_mount':c.tolist(),'translation_base_imu_m':lever.tolist(),
        'old_actual_position_difference_m':summary(old_actual_p),'old_actual_velocity_difference_mps':summary(old_actual_v),
        'old_actual_covariance_max_element_difference':summary(old_actual_cov),
        'legacy_kinematic_residual':residual(old_p,old_r,old_v,old_w),
        'candidate_kinematic_residual':residual(new_p,new_r,new_v,new_w),
        'legacy_candidate_velocity_difference_mps':summary(velocity_difference),
        'twist_covariance_frobenius_difference':summary(covariance_differences),
        'candidate_min_covariance_eigenvalue':float(min(min_eigen)),
        'invalid_covariance_samples':invalid,'calibration_verified':False,
        'input_covariance':'OpenVINS p_G・theta_I。upstreamの未公開相関・校正誤差は補っていない'}
    fields=['stamp']+[f'{mode}_{kind}_{axis}' for mode,kind in [('legacy','p'),('candidate','p'),('legacy','v'),('candidate','v')] for axis in 'xyz']
    fields += [f'{mode}_{kind}_variance_{i}' for mode,kind in [('legacy','pose'),('candidate','pose'),('legacy','twist'),('candidate','twist')] for i in range(6)]
    with (output/'samples.csv').open('w') as f:writer=csv.writer(f);writer.writerow(fields);writer.writerows(table)
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
    for axis,name in enumerate('xyz'):
        axes[axis].plot(times-times[0],old_v[:,axis],label='legacy v'+name,lw=.8)
        axes[axis].plot(times-times[0],new_v[:,axis],label='rigid candidate v'+name,lw=.8)
        axes[axis].set_ylabel('v'+name+' [m/s]');axes[axis].grid(alpha=.25);axes[axis].legend(fontsize=8)
    axes[-1].set_xlabel('time after audit reference [s]');fig.suptitle(bag.name+' — same raw input, different rigid transform');fig.tight_layout();fig.savefig(output/'velocity_comparison.png',dpi=150);plt.close(fig)
    (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bags',nargs='+',type=Path);p.add_argument('--output-dir',required=True,type=Path);p.add_argument('--sample-hz',type=float,default=5.)
    args=p.parse_args()
    if not math.isfinite(args.sample_hz) or args.sample_hz<=0:p.error('sample-hzは有限正数')
    args.output_dir.mkdir(parents=True,exist_ok=True);results=[]
    for bag in args.bags:
        try:result=analyze(bag,args.output_dir/bag.name,args.sample_hz)
        except Exception as exc:result={'bag':str(bag),'status':'error','reason':repr(exc)}
        results.append(result);print(json.dumps(result,ensure_ascii=False),flush=True)
    (args.output_dir/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    if any(r['status']!='ok' for r in results):raise SystemExit(1)

if __name__=='__main__':main()
