#!/usr/bin/env python3
"""Read-only MCAP VIO audit. Image features are offline proxies, not OpenVINS tracks.

Run in the Foxy container with pm_evaluation sourced. Repeat --bag for comparison.
CSV caches allow --plot-only without reading image payloads again.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages
from pm_evaluation.cli.plot_bag_trajectories import find_mcap_files

IMAGES = ['/oak/stereo/left/image_raw', '/oak/stereo/right/image_raw', '/oak/color/image_raw']
IMUS = ['/oak/imu/data', '/wit/imu']
ODOMS = ['/ov_msckf/odomimu', '/vio/odometry', '/wheel/odometry']


def key(topic):
    return topic.strip('/').replace('/', '_')


def stamp(msg, fallback):
    s = msg.header.stamp if hasattr(msg, 'header') else None
    return s.sec + s.nanosec * 1e-9 if s else fallback


def vec(v):
    return [v.x, v.y, v.z]


def save(path, rows, columns):
    with path.open('w') as f:
        w = csv.writer(f)
        w.writerow(columns.split(','))
        w.writerows(rows)


def load(out, name):
    return np.genfromtxt(out / (name + '.csv'), delimiter=',', names=True)


def gray(msg):
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    if msg.encoding in ('mono8', '8UC1'):
        return buf[:, :msg.width].copy()
    if msg.encoding in ('rgb8', 'bgr8'):
        a = buf[:, :msg.width*3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(a, cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY)
    raise ValueError('Unsupported encoding ' + msg.encoding)


def extract(bag, out, sample_hz):
    out.mkdir(parents=True, exist_ok=True)
    (out / 'frames').mkdir(exist_ok=True)
    meta = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    t0 = meta['starting_time']['nanoseconds_since_epoch'] * 1e-9
    (out / 'metadata.json').write_text(json.dumps(meta, indent=2))
    rows, timing = defaultdict(list), defaultdict(list)
    logs, parameters, diagnostics, static = [], [], [], {}
    last, last_frame, previous = {}, {}, {}
    selected = IMAGES + IMUS + ODOMS + ['/ov_msckf/poseimu', '/fix_velocity', '/rosout',
        '/parameter_events', '/diagnostics', '/tf_static', '/motor_state']
    cv2.setNumThreads(1)
    for path in find_mcap_files(bag):
        print('Extracting', bag.name, path.name, flush=True)
        for item in read_ros2_messages(path, topics=selected):
            topic, msg = item.channel.topic, item.ros_msg
            rec = item.log_time_ns * 1e-9
            ts = stamp(msg, rec)
            timing[topic].append([ts, rec, rec-ts])
            if topic in IMUS:
                rows[topic].append([ts, rec] + vec(msg.linear_acceleration) + vec(msg.angular_velocity))
            elif topic in ODOMS:
                p, v = msg.pose.pose, msg.twist.twist
                q = p.orientation
                rows[topic].append([ts, rec]+vec(p.position)+[q.x,q.y,q.z,q.w]+vec(v.linear)+vec(v.angular)+
                    [msg.pose.covariance[i] for i in [0,7,14,21,28,35]]+
                    [msg.twist.covariance[i] for i in [0,7,14]])
            elif topic == '/ov_msckf/poseimu':
                p = msg.pose.pose
                rows[topic].append([ts,rec]+vec(p.position)+[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
            elif topic == '/fix_velocity':
                rows[topic].append([ts,rec]+vec(msg.twist.twist.linear))
            elif topic in IMAGES:
                if ts-last.get(topic, -1e20) < 1/sample_hz:
                    continue
                last[topic] = ts
                a = gray(msg)
                # Same resolution for cross-bag image-quality metrics.
                a = cv2.resize(a, (640,400), interpolation=cv2.INTER_AREA)
                corners = cv2.goodFeaturesToTrack(a, 300, .01, 8)
                n = 0 if corners is None else len(corners)
                occupied = 0 if corners is None else len(set((int(x//128),int(y//80)) for x,y in corners[:,0]))
                survival, flow = np.nan, np.nan
                if topic in previous:
                    prev, pts = previous[topic]
                    if pts is not None:
                        nxt, status, err = cv2.calcOpticalFlowPyrLK(prev,a,pts,None)
                        back, st2, _ = cv2.calcOpticalFlowPyrLK(a,prev,nxt,None)
                        good = (status[:,0]>0)&(st2[:,0]>0)&(np.linalg.norm(back[:,0]-pts[:,0],axis=1)<1.5)
                        survival = float(good.mean())
                        if good.any():
                            flow = float(np.median(np.linalg.norm(nxt[good,0]-pts[good,0],axis=1)))
                previous[topic] = (a,corners)
                rows[topic].append([ts,rec,float(a.mean()),float(a.std()),float(cv2.Laplacian(a,cv2.CV_64F).var()),
                    float((a<10).mean()),float((a>245).mean()),n,occupied/25.,survival,flow,msg.width,msg.height])
                if ts-last_frame.get(topic,-1e20)>=15:
                    last_frame[topic]=ts
                    cv2.imwrite(str(out/'frames'/('%s_%07.2f.png'%(key(topic),ts-t0))),a)
            elif topic == '/rosout':
                logs.append({'t':ts-t0,'record_t':rec-t0,'name':msg.name,'level':msg.level,'msg':msg.msg})
            elif topic == '/parameter_events':
                parameters.append({'t':rec-t0,'node':msg.node,'new':repr(msg.new_parameters),'changed':repr(msg.changed_parameters)})
            elif topic == '/diagnostics':
                diagnostics.append({'t':ts-t0,'status':repr(msg.status)})
            elif topic == '/tf_static':
                for tf in msg.transforms:
                    static[tf.child_frame_id]={'parent':tf.header.frame_id,'transform':repr(tf.transform)}
            elif topic == '/motor_state':
                if len(rows[topic]) < 3:
                    rows[topic].append([ts,repr(msg)])
    for topic, data in rows.items():
        if topic in IMUS:
            columns='t,record_t,ax,ay,az,wx,wy,wz'
        elif topic in ODOMS:
            columns='t,record_t,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,var_x,var_y,var_z,var_roll,var_pitch,var_yaw,var_vx,var_vy,var_vz'
        elif topic in IMAGES:
            columns='t,record_t,mean,std,laplacian_var,dark_fraction,bright_fraction,corners,grid_coverage,lk_survival,lk_flow_px,width,height'
        elif topic == '/fix_velocity':
            columns='t,record_t,vx,vy,vz'
        elif topic == '/ov_msckf/poseimu':
            columns='t,record_t,x,y,z,qx,qy,qz,qw'
        else:
            columns='t,message'
        save(out/(key(topic)+'.csv'),data,columns)
    audit={}
    for topic,data in timing.items():
        a=np.array(data)
        save(out/(key(topic)+'_timing.csv'),a,'t,record_t,delay')
        d=np.diff(a[:,0])
        audit[topic]={'count':len(a),'first_header_from_bag_s':a[0,0]-t0,'last_header_from_bag_s':a[-1,0]-t0,
            'header_nonpositive_deltas':int((d<=0).sum()),'header_gap_p50_p95_max_s':np.percentile(d,[50,95,100]).tolist() if len(d) else [],
            'delay_p0_p50_p95_max_s':np.percentile(a[:,2],[0,50,95,100]).tolist()}
    for name,obj in [('audit',audit),('rosout',logs),('parameters',parameters),('diagnostics',diagnostics),('static_tf',static)]:
        (out/(name+'.json')).write_text(json.dumps(obj,indent=2))
    return out


def plot(out):
    meta=json.loads((out/'metadata.json').read_text())
    t0=meta['starting_time']['nanoseconds_since_epoch']*1e-9
    fig,axes=plt.subplots(5,1,figsize=(14,16),sharex=True)
    for topic in ODOMS+['/fix_velocity']:
        p=out/(key(topic)+'.csv')
        if not p.exists(): continue
        a=load(out,key(topic)); t=a['t']-t0
        axes[0].plot(t,np.sqrt(a['vx']**2+a['vy']**2+a['vz']**2),label=topic,lw=.8)
        if 'z' in a.dtype.names:
            axes[1].plot(t,a['z']-a['z'][0],label=topic,lw=.8)
    axes[0].set_yscale('symlog',linthresh=1)
    axes[0].set_ylabel('Speed norm [m/s]')
    axes[1].set_yscale('symlog',linthresh=1)
    axes[1].set_ylabel('Relative z [m]')
    for topic in IMUS:
        a=load(out,key(topic)); t=a['t']-t0
        axes[2].plot(t,np.sqrt(a['ax']**2+a['ay']**2+a['az']**2),label=topic,lw=.5)
        axes[3].plot(t,np.sqrt(a['wx']**2+a['wy']**2+a['wz']**2),label=topic,lw=.5)
    axes[2].set(ylabel='Accelerometer norm [m/s²]\n(gravity included)')
    axes[3].set(ylabel='Gyro norm [rad/s]')
    for topic in IMAGES[:2]:
        a=load(out,key(topic)); axes[4].plot(a['t']-t0,a['lk_survival'],label=topic,lw=.8)
    axes[4].set(ylabel='Offline LK survival\n(~0.5 s pairs)',xlabel='Seconds since bag recording start')
    for ax in axes: ax.grid(alpha=.3); ax.legend(fontsize=8)
    fig.suptitle(out.name); fig.tight_layout(); fig.savefig(out/'vio_timeline.png',dpi=150); plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(14,9),sharex=True)
    for topic in IMAGES:
        a=load(out,key(topic)); t=a['t']-t0
        for ax,field in zip(axes,['mean','laplacian_var','grid_coverage']): ax.plot(t,a[field],label=topic)
    for ax,field in zip(axes,['Mean gray','Laplacian variance (texture + blur)','Corner grid coverage']):
        ax.set_ylabel(field); ax.legend(fontsize=8); ax.grid(alpha=.3)
    axes[-1].set_xlabel('Seconds since bag recording start'); fig.tight_layout();fig.savefig(out/'image_quality.png',dpi=150);plt.close(fig)
    for topic in [IMAGES[0],IMAGES[2]]:
        files=sorted((out/'frames').glob(key(topic)+'_*.png'))
        fig,axes=plt.subplots(int(np.ceil(len(files)/4)),4,figsize=(16,3*int(np.ceil(len(files)/4))))
        for ax in np.ravel(axes): ax.axis('off')
        for ax,p in zip(np.ravel(axes),files):
            ax.imshow(cv2.imread(str(p),0),cmap='gray',vmin=0,vmax=255); ax.set_title(p.stem.split('_')[-1]+' s')
        fig.suptitle(out.name+' / '+topic);fig.tight_layout();fig.savefig(out/(key(topic)+'_contact_sheet.png'),dpi=130);plt.close(fig)


def startup_detail(bag, out, duration=45):
    """Inspect cached numeric signals and retrieve only early left-camera frames."""
    meta=json.loads((out/'metadata.json').read_text())
    t0=meta['starting_time']['nanoseconds_since_epoch']*1e-9
    shots=[]; last=-1e20
    for path in find_mcap_files(bag):
        done=False
        for item in read_ros2_messages(path,topics=[IMAGES[0]]):
            msg=item.ros_msg; t=stamp(msg,item.log_time_ns*1e-9)-t0
            if t>duration: done=True; break
            if t-last<2: continue
            last=t; a=gray(msg)
            shots.append((t,a))
        if done: break
    fig,axes=plt.subplots(int(np.ceil(len(shots)/4)),4,figsize=(16,3*int(np.ceil(len(shots)/4))))
    for ax in np.ravel(axes): ax.axis('off')
    for ax,(t,a) in zip(np.ravel(axes),shots):
        ax.imshow(a,cmap='gray',vmin=0,vmax=255);ax.set_title('%.2f s'%t)
    fig.suptitle(out.name+' / startup left images');fig.tight_layout();fig.savefig(out/'startup_frames.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(5,1,figsize=(13,13),sharex=True)
    o=load(out,'ov_msckf_odomimu'); ot=o['t']-t0
    for k in ['x','y','z']: axes[0].plot(ot,o[k]-o[k][0],label=k)
    axes[0].set(yscale='symlog',ylabel='OpenVINS position [m]')
    q=np.column_stack([o[k] for k in ['qx','qy','qz','qw']]); q/=np.linalg.norm(q,axis=1)[:,None]
    # Quaternion angular distance is independent of Hamilton/JPL convention.
    angle=np.degrees(2*np.arccos(np.clip(abs(q@q[0]),0,1)))
    axes[1].plot(ot,angle,label='OpenVINS angle from first pose')
    for topic in IMUS:
        a=load(out,key(topic)); t=a['t']-t0
        norm=np.sqrt(sum(a[k]**2 for k in ['wx','wy','wz']))
        dt=np.diff(a['t']); integrated=np.r_[0,np.cumsum(.5*(norm[1:]+norm[:-1])*dt)]
        integrated-=np.interp(ot[0],t,integrated)
        axes[1].plot(t,np.degrees(integrated),label=topic+' integral |gyro|')
        axes[2].plot(t,np.sqrt(sum(a[k]**2 for k in ['ax','ay','az'])),label=topic,lw=.7)
    axes[1].set_ylabel('Angle / gyro path length [deg]')
    axes[2].set_ylabel('Accel norm incl. gravity [m/s²]')
    a=load(out,key(IMAGES[0]));t=a['t']-t0
    axes[3].plot(t,a['mean'],label='Mean gray')
    axes[3].plot(t,a['bright_fraction']*255,label='Saturated fraction ×255')
    axes[3].set_ylabel('Image exposure proxy')
    axes[4].plot(t,a['lk_survival'],label='Offline LK survival ~0.5 s')
    axes[4].plot(t,a['grid_coverage'],label='Offline corner grid coverage')
    axes[4].set(ylabel='Fraction',xlabel='Seconds since bag recording start')
    for ax in axes: ax.set_xlim(10,duration);ax.legend(fontsize=8);ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(out/'startup_diagnosis.png',dpi=150);plt.close(fig)


def summarize(out):
    meta=json.loads((out/'metadata.json').read_text());t0=meta['starting_time']['nanoseconds_since_epoch']*1e-9
    result={'bag':out.name,'time_origin':'bag recording start; not launch start','t0':t0}
    for topic in ODOMS:
        a=load(out,key(topic));s=np.sqrt(sum(a[k]**2 for k in ['vx','vy','vz']))
        item={'speed_p50_p95_max_mps':np.percentile(s,[50,95,100]).tolist(),
              'position_range_xyz_m':[float(np.ptp(a[k])) for k in ['x','y','z']]}
        for threshold in [.2,2,5,10]:
            ids=np.flatnonzero(s>threshold)
            item['first_speed_over_%g_mps_s'%threshold]=float(a['t'][ids[0]]-t0) if len(ids) else None
        result[topic]=item
    for topic in IMUS:
        a=load(out,key(topic));acc=np.column_stack([a[k] for k in ['ax','ay','az']]);w=np.column_stack([a[k] for k in ['wx','wy','wz']]);t=a['t']-t0
        item={}
        for lo,hi in [(13,18),(18,23),(23,30)]:
            mask=(t>=lo)&(t<hi)
            item['%d_%d_s'%(lo,hi)]={'mean_accel_xyz':acc[mask].mean(axis=0).tolist(),
                'accel_vector_std_norm':float(np.linalg.norm(acc[mask].std(axis=0))),
                'accel_norm_max':float(np.linalg.norm(acc[mask],axis=1).max()),
                'gyro_norm_max':float(np.linalg.norm(w[mask],axis=1).max())}
        result[topic]=item
    for topic in IMAGES:
        a=load(out,key(topic)); result[topic]={k:float(np.nanmedian(a[k])) for k in ['mean','laplacian_var','grid_coverage','lk_survival']}
    (out/'diagnosis_summary.json').write_text(json.dumps(result,indent=2))


def plot_replays(out):
    meta=json.loads((out/'metadata.json').read_text());t0=meta['starting_time']['nanoseconds_since_epoch']*1e-9
    fig,axes=plt.subplots(4,1,figsize=(13,12),sharex=True)
    series={'recorded':load(out,'ov_msckf_odomimu')}
    for name in ['replay_current_config','replay_frozen_calibration']:
        series[name]=load(out/name,'odom')
    report={}
    colors={'recorded':'tab:blue','replay_current_config':'tab:orange','replay_frozen_calibration':'tab:green'}
    for name,a in series.items():
        t=a['t']-t0;mask=(t>=19.5)&(t<=44.5)
        speed=np.sqrt(sum(a[k]**2 for k in ['vx','vy','vz']))
        xyz=np.c_[a['x'],a['y'],a['z']]
        show=(t>=18)&(t<=45)
        axes[0].plot(t[show],speed[show],label=name,color=colors[name])
        axes[1].plot(t[show],np.linalg.norm(xyz[show]-xyz[0],axis=1),label=name,color=colors[name])
        report[name]={'samples':len(a),'speed_max_19p5_44p5_mps':float(speed[mask].max()),
            'displacement_at_44p5_m':float(np.linalg.norm(xyz[np.argmin(abs(t-44.5))]-xyz[0]))}
        if name!='recorded':
            s=np.loadtxt(out/name/'state_estimate.txt')
            axes[2].plot(s[:,0]-t0,np.linalg.norm(s[:,11:14],axis=1),label=name,color=colors[name])
            axes[3].plot(s[:,0]-t0,s[:,17]*1000,label=name,color=colors[name])
            report[name]['final_bg_xyz_rad_s']=s[-1,11:14].tolist()
            report[name]['final_cam_imu_timeoffset_s']=float(s[-1,17])
    axes[0].set_yscale('symlog',linthresh=.2);axes[0].set_ylabel('Speed [m/s]')
    axes[1].set_yscale('symlog',linthresh=.2);axes[1].set_ylabel('Displacement [m]')
    axes[2].set_ylabel('Estimated gyro bias norm [rad/s]')
    axes[3].set(ylabel='Estimated cam/IMU offset [ms]',xlabel='Seconds since original bag start')
    for ax in axes:ax.set_xlim(18,45);ax.grid(alpha=.3);ax.legend(fontsize=8)
    fig.suptitle('Controlled startup replays: current binary/config, rate=0.5; historical internal state unavailable')
    fig.tight_layout();fig.savefig(out/'startup_replay_comparison.png',dpi=150);plt.close(fig)
    (out/'startup_replay_summary.json').write_text(json.dumps(report,indent=2))


def solve_rotation(source, target):
    dot=np.sum(source*target)
    cross=np.sum(source[:,0]*target[:,1]-source[:,1]*target[:,0])
    return np.arctan2(cross,dot)


def plot_offline_overlay(out, comparison, replay_tag, fit_start_s, fit_end_s):
    from pyproj import CRS,Transformer
    meta=json.loads((out/'metadata.json').read_text());t0=meta['starting_time']['nanoseconds_since_epoch']*1e-9
    vio=load(out/('replay_'+replay_tag),'odom')
    fix=np.genfromtxt(comparison/'fix.csv',delimiter=',',names=True)
    wheel=np.genfromtxt(comparison/'wheel_gyro_aligned_gnss_offset_137p7.csv',delimiter=',',names=True)
    local=CRS.from_proj4('+proj=aeqd +lat_0=%s +lon_0=%s +datum=WGS84 +units=m'%(fix['lat'][0],fix['lon'][0]))
    to_local=Transformer.from_crs(4326,local,always_xy=True);to_map=Transformer.from_crs(local,3857,always_xy=True)
    gx,gy=to_local.transform(fix['lon'],fix['lat']);g=np.c_[gx,gy]
    # Translation and yaw only, solved on an explicitly reported early window. No scale fitting.
    fit_t=fix['t'][(fix['t']>=t0+fit_start_s)&(fix['t']<=t0+fit_end_s)]
    source=np.c_[np.interp(fit_t,vio['t'],vio['x']),np.interp(fit_t,vio['t'],vio['y'])]
    target=np.c_[np.interp(fit_t,fix['t'],gx),np.interp(fit_t,fix['t'],gy)]
    sc=source-source.mean(0);tc=target-target.mean(0);angle=solve_rotation(sc,tc)
    R=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    translation=target.mean(0)-source.mean(0)@R.T
    vxy=np.column_stack((vio['x'],vio['y']))@R.T+translation
    vm=np.column_stack(to_map.transform(vxy[:,0],vxy[:,1]))
    gm=np.column_stack(to_map.transform(g[:,0],g[:,1]))
    wm=np.column_stack(to_map.transform(wheel['east_m'],wheel['north_m']))
    stable_end=t0+fit_end_s
    fix_common=(fix['t']>=max(vio['t'][0],wheel['t'][0]))&(fix['t']<=min(vio['t'][-1],wheel['t'][-1]))
    vt=fix['t'][fix_common];vp=np.c_[np.interp(vt,vio['t'],vxy[:,0]),np.interp(vt,vio['t'],vxy[:,1])]
    gp=np.c_[np.interp(vt,fix['t'],gx),np.interp(vt,fix['t'],gy)];err=np.linalg.norm(vp-gp,axis=1)
    prefix=fix_common&(fix['t']<=stable_end)
    pt=fix['t'][prefix];pp=np.c_[np.interp(pt,vio['t'],vxy[:,0]),np.interp(pt,vio['t'],vxy[:,1])]
    pg=np.c_[np.interp(pt,fix['t'],gx),np.interp(pt,fix['t'],gy)];perr=np.linalg.norm(pp-pg,axis=1)
    report={'replay_tag':replay_tag,'alignment':'translation+yaw fit, no scale',
        'fit_window_seconds_from_bag_start':[fit_start_s,fit_end_s],'fit_rotation_deg':float(np.degrees(angle)),
        'stable_prefix_gnss_rmse_m':float(np.sqrt(np.mean(perr**2))),
        'full_available_gnss_rmse_m':float(np.sqrt(np.mean(err**2))),
        'full_available_end_s':float(vio['t'][-1]-t0)}
    (out/('offline_overlay_'+replay_tag+'.json')).write_text(json.dumps(report,indent=2))
    colors={'gnss':'black','wheel':'tab:purple','vio':'tab:green'}
    cache=np.load(comparison/'basemap.npz');image,extent=cache['image'],cache['extent']
    gmask=(fix['t']>=wheel['t'][0])&(fix['t']<=wheel['t'][-1])
    for name,end,full in [('stable_prefix',stable_end,False),('map',vio['t'][-1],False),('full',vio['t'][-1],True)]:
        fig,ax=plt.subplots(figsize=(11,9))
        if not full:ax.imshow(image,extent=extent,zorder=0);ax.text(.01,.01,'© OpenStreetMap contributors',transform=ax.transAxes,fontsize=7)
        plot_gmask=gmask&(fix['t']<=end) if name=='stable_prefix' else gmask
        plot_wmask=wheel['t']<=end if name=='stable_prefix' else np.ones(len(wheel),dtype=bool)
        ax.plot(gm[plot_gmask,0],gm[plot_gmask,1],'k.',ms=3,label='GNSS (not ground truth)')
        ax.plot(wm[plot_wmask,0],wm[plot_wmask,1],color=colors['wheel'],lw=1.5,label='wheel vx + Wit wz EKF (+137.7°)')
        use=vio['t']<=end;ax.plot(vm[use,0],vm[use,1],color=colors['vio'],lw=1.3,label='offline OpenVINS, fixed calibration')
        if full:
            points=np.vstack([gm[plot_gmask],wm[plot_wmask],vm[use]]);lo=points.min(0);hi=points.max(0);margin=max((hi-lo).max()*.03,5)
            ax.set_xlim(lo[0]-margin,hi[0]+margin);ax.set_ylim(lo[1]-margin,hi[1]+margin)
        else:
            points=np.vstack([gm[plot_gmask],wm[plot_wmask],vm[use]]) if name=='stable_prefix' else gm[plot_gmask]
            lo=points.min(0);hi=points.max(0);margin=max((hi-lo).max()*.08,2)
            ax.set_xlim(lo[0]-margin,hi[0]+margin);ax.set_ylim(lo[1]-margin,hi[1]+margin)
        ax.set_aspect('equal');ax.grid(alpha=.25);ax.legend();ax.set_xlabel('Web Mercator East [projected m]');ax.set_ylabel('Web Mercator North [projected m]')
        title='Offline VIO overlay: '+name.replace('_',' ')+'\nyaw/translation fit %.1f–%.1f s; no scale fit'%(fit_start_s,fit_end_s)
        ax.set_title(title);fig.tight_layout();fig.savefig(out/('offline_vio_overlay_'+name+'.png'),dpi=160);plt.close(fig)


def replay_startup(bag, out, config, freeze, duration, rate, replay_tag=None):
    """Controlled counterfactual with current source/config; not historical reproduction."""
    import os
    import subprocess
    import signal
    import time
    from mcap.reader import make_reader
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from ament_index_python.packages import get_package_prefix
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    os.environ['ROS_DOMAIN_ID']='91'
    os.environ['ROS_LOCALHOST_ONLY']='1'
    default_tag = 'frozen_calibration' if freeze else 'current_config'
    run=out/('replay_' + (replay_tag or default_tag))
    if run.exists(): raise RuntimeError('Use new output directory for a new replay: '+str(run))
    run.mkdir(parents=True)
    run=run.resolve()
    meta=json.loads((out/'metadata.json').read_text());t0=meta['starting_time']['nanoseconds_since_epoch']*1e-9
    topics=IMAGES[:2]+[IMUS[0]];events=[];types={}
    for path in find_mcap_files(bag):
        done=False
        with path.open('rb') as f:
            for schema,channel,message in make_reader(f).iter_messages(topics=topics):
                if message.log_time*1e-9>t0+duration: done=True;break
                types[channel.topic]=get_message(schema.name)
                events.append((message.log_time,channel.topic,message.data))
        if done: break
    events.sort(key=lambda x:x[0])
    rclpy.init();node=rclpy.create_node('vio_startup_audit_replay')
    pubs={t:node.create_publisher(types[t],t,100) for t in topics}
    cp=node.create_publisher(Clock,'/clock',10)
    output=[]
    def cb(m):
        p=m.pose.pose;q=p.orientation;v=m.twist.twist
        output.append([stamp(m,0)]+vec(p.position)+[q.x,q.y,q.z,q.w]+vec(v.linear)+vec(v.angular))
    sub=node.create_subscription(Odometry,'/ov_msckf/odomimu',cb,1000)
    cmd=[str(Path(get_package_prefix('ov_msckf'))/'lib/ov_msckf/run_subscribe_msckf'),
         '--ros-args','-r','__ns:=/ov_msckf','-p','config_path:='+str(config.resolve()),
         '-p','use_sim_time:=true','-p','verbosity:=DEBUG','-p','save_total_state:=true',
         '-p','filepath_est:='+str(run/'state_estimate.txt'),'-p','filepath_std:='+str(run/'state_std.txt'),
         '-p','record_timing_information:=true','-p','record_timing_filepath:='+str(run/'timing.txt')]
    if freeze:
        for k in ['calib_cam_extrinsics','calib_cam_intrinsics','calib_cam_timeoffset','calib_imu_intrinsics','calib_imu_g_sensitivity']:
            cmd+=['-p',k+':=false']
    (run/'run.json').write_text(json.dumps({'command':cmd,'duration_from_bag_start':duration,'rate':rate,
        'warning':'Current local config and binary, not proof of historical runtime equality','bag':str(bag.resolve())},indent=2))
    log=(run/'console.log').open('w');proc=subprocess.Popen(cmd,stdout=log,stderr=log)
    try:
        deadline=time.monotonic()+20
        while any(p.get_subscription_count()==0 for p in pubs.values()):
            if proc.poll() is not None or time.monotonic()>deadline: raise RuntimeError('OpenVINS startup failed; inspect console.log')
            rclpy.spin_once(node,timeout_sec=.05)
        first,last=events[0][0],events[-1][0];clock=Clock();index=0;wall=time.monotonic();report=0
        for now in range(first,last+10000001,10000000):
            due=wall+(now-first)*1e-9/rate
            while time.monotonic()<due: rclpy.spin_once(node,timeout_sec=min(.002,max(0,due-time.monotonic())))
            clock.clock.sec=now//1000000000;clock.clock.nanosec=now%1000000000;cp.publish(clock)
            while index<len(events) and events[index][0]<=now:
                _,topic,data=events[index];pubs[topic].publish(deserialize_message(data,types[topic]));index+=1
            rclpy.spin_once(node,timeout_sec=0)
            if (now-first)*1e-9>=report:
                print(run.name,'bag time',round(now*1e-9-t0,1),'outputs',len(output),flush=True);report+=10
            if proc.poll() is not None: raise RuntimeError('OpenVINS exited during replay')
        until=time.monotonic()+2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=10)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close();node.destroy_node();rclpy.shutdown()
        save(run/'odom.csv',output,'t,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--sample-hz',type=float,default=2)
    p.add_argument('--plot-only',action='store_true')
    p.add_argument('--startup-only',action='store_true',help='Use cached CSV and retrieve only first 45 s images')
    p.add_argument('--replay-startup',action='store_true')
    p.add_argument('--plot-replays',action='store_true')
    p.add_argument('--plot-offline-overlay',action='store_true')
    p.add_argument('--comparison-results',type=Path)
    p.add_argument('--overlay-replay-tag',default='full_frozen_calibration')
    p.add_argument('--fit-start-s',type=float,default=48)
    p.add_argument('--fit-end-s',type=float,default=63.5)
    p.add_argument('--config',type=Path,default=Path('src/pm_config/config/oak_d_s2/estimator_config1.yaml'))
    p.add_argument('--freeze-calibration',action='store_true')
    p.add_argument('--replay-duration',type=float,default=45)
    p.add_argument('--rate',type=float,default=.5)
    p.add_argument('--replay-tag',help='Reusable output tag, e.g. full_frozen_calibration')
    args=p.parse_args()
    for bag in args.bag:
        out=args.output/bag.name
        if args.plot_offline_overlay:
            if args.comparison_results is None:p.error('--comparison-results is required')
            plot_offline_overlay(out,args.comparison_results,args.overlay_replay_tag,args.fit_start_s,args.fit_end_s)
            continue
        if args.plot_replays:
            plot_replays(out)
            continue
        if args.replay_startup:
            replay_startup(bag,out,args.config,args.freeze_calibration,args.replay_duration,args.rate,args.replay_tag)
            continue
        if args.startup_only:
            startup_detail(bag,out)
            summarize(out)
            continue
        if not args.plot_only:
            if (out/'audit.json').exists(): p.error('Existing extraction: use --plot-only or a new output directory')
            extract(bag,out,args.sample_hz)
        plot(out)
        summarize(out)
        print('Saved',out,flush=True)


if __name__=='__main__': main()
