"""旧版を複製した２回再生版。第１段でVOを共有生成し、第２段で全EKF条件を共有再生する。"""
import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import tempfile

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from pm_evaluation.localization_batch_io import (discover, discover_incomplete, metadata, read_snapshot,
                                                fingerprint, MODES, header_sec, TOPICS)
from pm_evaluation.localization_batch_plan import create_job, config_values
from pm_evaluation.localization_batch_plot import (load, fixes, map_cache, placement,
                                                  mapped, draw, closest, pose_series, figure_notes, annotate_figure)
from pm_localization.vio_odom_adapter_node import quat_to_rot


# 両版で初期方位の規約を共有し、global補正TFの混入を防ぐ。
from pm_evaluation.localization_heading import initial_heading as heading, magnetic_seed_yaw


def input_report(data, output):
    """途絶とゲート診断を集計。診断件数を通過データ件数と混同しない。"""
    report={}
    for topic,rows in data.items():
        times=[header_sec(m) for _,m in rows if hasattr(m,'header')]
        gaps=np.diff(times) if len(times)>1 else np.array([])
        report[topic]={'messages':len(rows),'first_header':min(times) if times else None,
                       'last_header':max(times) if times else None,
                       'max_header_gap_sec':float(np.max(gaps)) if len(gaps) else None,
                       'mean_header_hz':(len(times)-1)/(max(times)-min(times)) if len(times)>1 and max(times)>min(times) else None,
                       'header_reversals':int(np.sum(gaps<0)),
                       'note':'診断topicの件数は状態通知数であり通過率ではない'}
    if data.get('/vio/odometry'):
        received=len(data['/vio/odometry'])
        report['gate_recorded_counts']={
            'adapter_output':received,
            'twist_output':len(data.get('/vio/odometry/twist_gated',[])),
            'height_output':len(data.get('/vio/odometry/gated',[])),
            'note':'実データ記録件数。診断通知件数ではない。recorder自身の取りこぼしは別途考慮'}
    (output/'input_topics.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report


def render_recorded(data, gps, background, output, snapshot=None):
    transform,note=placement(data,gps)
    result={'local':np.empty((0,13)),'global':np.empty((0,13))}
    if transform:
        for label,topic in [('local','/odometry/local'),('global','/odometry/global')]:
            rows=data.get(topic,[])
            if rows:result[label]=mapped(rows,transform,rows[0][1].header.frame_id)
    else:
        # 配置不能でも元座標CSVを残すが、東北座標として地図に偽装しない。
        for label,topic in [('local','/odometry/local'),('global','/odometry/global')]:
            a=pose_series(data.get(topic,[]))
            if len(a):np.savetxt(output/(label+'_native.csv'),a,delimiter=',')
    # 保存parameterがあれば、その試行で実際に指定されたEKF入力を確認する。
    import re
    expected=['/odometry/local','/odometry/global']
    for name,spec in (snapshot or {}).get('snapshots',{}).items():
        if 'ekf' not in name:continue
        for key,value in spec.get('values',{}).items():
            if (re.fullmatch(r'(odom|imu|twist)\d+',key) and isinstance(value,str)
                    and any(spec.get('values',{}).get(key+'_config',[True]))):expected.append(value)
    notes=figure_notes(data,sorted(set(expected)),('/fix',),
        ('Recorded inputs: availability does not prove fusion; unrecorded inputs cannot be assessed.',))
    if not transform:notes.append('MAP PLACEMENT FAILED: '+note)
    draw(output,gps,result,background,'Recorded local / global',notes)
    (output/'placement.json').write_text(json.dumps({'method':note},ensure_ascii=False,indent=2))


def state_plots(data, output, notes=None):
    """速度・分散・内部加速度を保存。無受信区間を外挿で補わない。"""
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
    t0=None
    for label,topic in [('local','/odometry/local'),('global','/odometry/global')]:
        a=pose_series(data.get(topic,[]))
        if not len(a):continue
        if t0 is None:t0=a[0,0]
        axes[0].plot(a[:,0]-t0,a[:,5],label=label+' vy')
        axes[1].semilogy(a[:,0]-t0,np.maximum(a[:,11],1e-12),label=label+' var(vy)')
    accelerations={}
    for topic in ('/batch/local/accel','/batch/global/accel'):
        rows=[]
        for _,m in data.get(topic,[]):
            v=m.accel.accel.linear;rows.append([header_sec(m),v.x,v.y,v.z])
        if rows:
            a=np.array(rows);t0=a[0,0] if t0 is None else t0
            axes[2].plot(a[:,0]-t0,a[:,2],label=topic+' ay')
            np.savetxt(output/(topic.split('/')[-2]+'_acceleration.csv'),a,delimiter=',',header='stamp,ax,ay,az',comments='')
    axes[0].set_ylabel('vy [m/s]');axes[1].set_ylabel('variance [(m/s)²]');axes[2].set(ylabel='ay [m/s²]',xlabel='time [s]')
    for ax in axes:
        ax.grid()
        if ax.get_legend_handles_labels()[0]:ax.legend(fontsize=8)
    annotate_figure(fig,notes or figure_notes(data,('/odometry/local','/odometry/global'),('/odometry/local','/odometry/global')));fig.savefig(output/'states.png',dpi=140);plt.close(fig)


def output_quality(data):
    """生成されたという理由だけで数値的に不正な推定を成功扱いしない。"""
    result={};invalid=False;warning=False
    for topic in ('/odometry/local','/odometry/global'):
        bad=0;first_bad=None;maximum_speed=0.
        for _,msg in data.get(topic,[]):
            v=msg.twist.twist.linear
            velocity=np.array([v.x,v.y,v.z]);speed=float(np.linalg.norm(velocity))
            if np.isfinite(speed):maximum_speed=max(maximum_speed,speed)
            malformed=not np.isfinite(velocity).all()
            for values in (msg.pose.covariance,msg.twist.covariance):
                matrix=np.asarray(values).reshape(6,6)
                if not np.isfinite(matrix).all():malformed=True;continue
                symmetric=(matrix+matrix.T)/2
                if (np.diag(symmetric).min() < -1e-9 or
                    np.linalg.eigvalsh(symmetric).min() < -1e-8*max(1.,np.linalg.norm(symmetric))):
                    malformed=True
            if malformed:
                bad+=1
                if first_bad is None:first_bad=header_sec(msg)
        result[topic]=dict(invalid_covariance_or_velocity_messages=bad,
            first_invalid_stamp=first_bad,maximum_speed_mps=maximum_speed,
            speed_warning=maximum_speed>3.,note='3 m/s超は要確認。速度だけでは数値破綻と断定しない')
        invalid|=bad>0;warning|=maximum_speed>3.
    return dict(numerically_invalid=invalid,speed_warning=warning,topics=result)



def native_ready(folder):
    """VO条件ごとの完了判定。入力hubと独立してraw結果を再利用する。"""
    from pm_evaluation import localization_batch_cache as cache
    try:
        saved=json.loads((Path(folder)/'complete.json').read_text())
        return saved.get('status')=='complete' and saved.get('storage')==cache.source_identity(Path(folder)/'raw_bag')
    except (OSError,ValueError,KeyError):return False


def main():
    from pm_evaluation import localization_batch_cache as cache
    from pm_evaluation.localization_batch_twopass import (COMMON_TOPICS,RAW_TOPIC,variant_topic,
                                                        attach_hub,build_hub,run_worker)
    from pm_evaluation.localization_vio_replay import configuration,configuration_files,SENSOR_TOPICS
    from pm_evaluation.localization_batch_twopass_parallel import run_shared
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_root',type=Path)
    parser.add_argument('--output-root',type=Path)
    parser.add_argument('--results-root',type=Path)
    parser.add_argument('--stages',nargs='+',choices=['recorded','historical','comparison','sources'],default=['recorded','historical','comparison','sources'])
    parser.add_argument('--modes',nargs='+',choices=MODES,default=list(MODES))
    parser.add_argument('--magnetic',nargs='+',choices=['on','off'],default=['on','off'])
    parser.add_argument('--bag',action='append')
    parser.add_argument('--basemap',type=Path)
    parser.add_argument('--no-map',action='store_true')
    parser.add_argument('--duration',type=float)
    parser.add_argument('--rate',type=float,default=1.)
    parser.add_argument('--domain',type=int,default=192)
    parser.add_argument('--initial-yaw-deg',type=float)
    parser.add_argument('--vio-variants',nargs='+',type=lambda v:{'online_on':'online-cam-calib_on','online_off':'online-cam-calib_off'}.get(v,v),
                        choices=['recorded','online-cam-calib_on','online-cam-calib_off'],default=['online-cam-calib_on','online-cam-calib_off'])
    parser.add_argument('--vio-config',type=Path)
    parser.add_argument('--wheel-per-mode',action='store_true')
    parser.add_argument('--graphs-per-domain',type=int,default=5,choices=range(1,6),help='DDS接続数を抑えるdomainあたりgraph数。再生は共有のまま')
    parser.add_argument('--max-ekf-graphs',type=int,default=64,help='全条件の同時graph数上限。超過時は再生回数を増やさず中止')
    parser.add_argument('--plan-only',action='store_true')
    parser.add_argument('--keep-shared-bags',action='store_true',help='分離照合後も共有中間bagを保持する（既定は削除）')
    parser.add_argument('--mcap-compression',choices=['zstd','none'],default='zstd',help='ROS recorderのMCAP chunk圧縮。Python生成bagは従来どおりZSTD')
    parser.add_argument('--resume',dest='resume',action='store_true',default=True)
    parser.add_argument('--no-resume',dest='resume',action='store_false')
    args=parser.parse_args()
    if not args.input_root.is_dir():parser.error('入力directoryなし')
    if not math.isfinite(args.rate) or args.rate<=0:parser.error('rateは有限正数')
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration<=0):parser.error('durationは有限正数')
    if not 1<=args.domain<=232 or args.max_ekf_graphs<1:parser.error('domain/graph上限が不正')
    if args.initial_yaw_deg is not None and not math.isfinite(args.initial_yaw_deg):parser.error('初期yawは有限値')
    args.vio_variants=list(dict.fromkeys(args.vio_variants));args.magnetic=list(dict.fromkeys(args.magnetic))
    root=args.input_root.resolve()
    workspace=next((p for p in [Path.cwd(),*Path.cwd().parents] if (p/'src/pm_evaluation').is_dir()),None)
    if args.results_root:results=args.results_root.resolve()
    elif workspace:results=workspace/'src/pm_evaluation/results'
    else:parser.error('--results-rootを指定')
    output=(args.output_root or results/(root.name+'_batch_twopass')).resolve()
    if output==root or output in root.parents:parser.error('出力で入力階層を覆えない')
    output.mkdir(parents=True,exist_ok=True)
    complete_bags=discover(root,excluded=[output])
    incomplete_bags=discover_incomplete(root,excluded=[output])
    # metadataなしの候補も一覧へ含め、未検出を「先頭失敗で中断」と誤認させない。
    # 不完全なstorageを修復・再生せず、bag単位の失敗として記録して次へ進む。
    bags=sorted(set(complete_bags+incomplete_bags))
    if args.bag:bags=[p for p in bags if p.name in args.bag or str(p.relative_to(root)) in args.bag]
    if not bags:parser.error('解析対象のmetadataまたはbag storageなし')
    print('対象bag候補: '+str(len(bags))+'件（metadataなし: '+str(sum(b in incomplete_bags for b in bags))+'件）',flush=True)
    config=Path(get_package_share_directory('pm_config'))/'config'
    code=cache.stage_code_files();package=Path(__file__).resolve().parent.parent
    backend=[Path(__file__),package/'localization_batch_twopass.py',package/'localization_batch_twopass_parallel.py',
             package/'cli/batch_localization_twopass_worker.py',package/'cli/batch_vio_twopass_worker.py',package/'localization_heading.py']
    numerical_code=code['ekf']+backend[1:4]
    render_code=code['render']+[Path(__file__)]
    from ament_index_python.packages import get_package_prefix
    native=Path(get_package_prefix('ov_msckf'))/'lib'
    vo_code=code['vo']+[backend[1],backend[4],native/'ov_msckf/run_subscribe_msckf']+list(native.glob('libov*.so'))
    all_results=[]
    for bag_index,bag in enumerate(bags,1):
        relative=bag.relative_to(root) if bag!=root else Path(bag.name)
        target=output/relative
        result=dict(bag=str(bag),relative=str(relative),jobs=[],replays=dict(vo=0,ekf=0),backend='two_pass')
        all_results.append(result);print('解析開始: '+str(relative)+' ('+str(bag_index)+'/'+str(len(bags))+')',flush=True)
        stamp=cache.analysis_timestamp().strftime('%Y%m%d_%H%M');folders={}
        def folder_for(parent,key,ready=None):
            lookup=(str(parent),key)
            if lookup not in folders:folders[lookup]=cache.analysis_directory(parent,key,stamp,args.resume,ready)
            return folders[lookup]
        try:
            target.mkdir(parents=True,exist_ok=True)
            if not (bag/'metadata.yaml').is_file():
                raise ValueError('metadata.yamlなし。記録未完了・コピー不足などを確認してください（自動修復せずスキップ）')
            lowband=bag
            if not args.plan_only:
                lowband,report=cache.extract_lowband(bag,target/'cache/lowband');result['lowband_cache']=report
            data=load(lowband);gps,origin=fixes(data);input_report(data,target)
            (target/'available_topics.json').write_text(json.dumps(metadata(bag)[0],ensure_ascii=False,indent=2))
            if args.plan_only:background=None;map_status='plan-only'
            else:background,map_status=map_cache(gps,origin,target/'basemap.npz',args.basemap,args.no_map)
            result['map']=map_status
            presentation=cache.render_key(cache.record_digest(lowband),target/'basemap.npz' if background else None,render_code)
            snapshot=read_snapshot(bag)
            if not args.plan_only:
                gnss=target/'gnss'
                if cache.render_needed(gnss,presentation,['xy_full.png','z.png'],args.resume):
                    draw(gnss,gps,{},background,bag.name+' GNSS');cache.mark_render(gnss,presentation)
                if gps:
                    shutil.copy2(gnss/'xy_full.png',bag/'gnss_trajectory_osm.png')
                    shutil.copy2(gnss/'xy_full.png',target/'gnss_trajectory_osm.png')
                if 'recorded' in args.stages:
                    rec=target/'recorded';rec.mkdir(exist_ok=True)
                    if cache.render_needed(rec,presentation,['xy_full.png','z.png','states.png'],args.resume):
                        render_recorded(data,gps,background,rec,snapshot)
                        state_plots(data,rec,json.loads((rec/'plot_annotations.json').read_text()));cache.mark_render(rec,presentation)
            if set(args.stages)=={'recorded'}:
                result['status']='planned' if args.plan_only else 'complete';continue
            for topic in ('/wheel/odometry','/wit/imu','/tf_static'):
                if not data.get(topic):raise ValueError('必要入力なし: '+topic)
            yaw,offset,method=heading(data,snapshot,config,None if args.initial_yaw_deg is None else math.radians(args.initial_yaw_deg))
            result['heading']=dict(method,recorded_yaw_rad=yaw,offset_deg=offset)
            (target/'recorded_parameters.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2))
            (target/'implementation.json').write_text(json.dumps(cache.implementation(backend+numerical_code+vo_code),indent=2))
            available={};pending=[];vo_folders={};vo_errors={}
            # 第１段階: 全ての未計算VOを用意してから一括起動する。条件ごとの再生はしない。
            for variant in args.vio_variants:
                if variant=='recorded':
                    if data.get(RAW_TOPIC):available[variant]=lowband
                    else:vo_errors[variant]='記録raw VIOなし'
                    continue
                try:
                    files=configuration_files(bag,args.vio_config)
                    key=cache.vio_key(cache.source_identity(bag),variant,args.rate,args.duration,files,vo_code)
                    vf=folder_for(target/'vio_replay'/variant,key,native_ready);vo_folders[variant]=vf
                    cfg=configuration(bag,vf/'config',variant,args.vio_config)
                    if args.resume and native_ready(vf):
                        available[variant]=vf/'raw_bag';status='cached'
                    elif args.plan_only:
                        available[variant]=vf/'raw_bag';status='planned'
                    else:
                        counts={v['topic_metadata']['name']:v['message_count'] for v in metadata(bag)[0]['topics_with_message_count']}
                        if not all(counts.get(t,0)>0 for t in SENSOR_TOPICS):raise ValueError('左右画像またはOAK IMUなし')
                        pending.append(dict(variant=variant,folder=str(vf),config=str(cfg)));status='pending'
                    result.setdefault('vo_cache',{})[variant]=dict(key=key,status=status)
                except Exception as exc:vo_errors[variant]=str(exc)
            if pending:
                group=folder_for(target/'pass1_vo',cache.digest([p['folder'] for p in pending]))
                spec=dict(bag=str(bag),rate=args.rate,duration=args.duration,variants=pending)
                (group/'job.json').write_text(json.dumps(spec,indent=2));result['replays']['vo']=1
                print('  PASS 1: VO '+', '.join(p['variant'] for p in pending)+' を共有再生',flush=True)
                fatal=None
                try:run_worker(['ros2','run','pm_evaluation','batch_vio_twopass_worker',str(group/'job.json')],group,args.domain)
                except Exception as exc:fatal=str(exc)
                for p in pending:
                    vf=Path(p['folder']);variant=p['variant']
                    delivery=json.loads((vf/'delivery.json').read_text()) if (vf/'delivery.json').exists() else {}
                    error=fatal or delivery.get('error')
                    if not delivery.get('output',{}).get(RAW_TOPIC):error=error or 'raw VIO出力なし'
                    if error:vo_errors[variant]=error
                    else:
                        available[variant]=vf/'raw_bag'
                        (vf/'complete.json').write_text(json.dumps(dict(status='complete',storage=cache.source_identity(vf/'raw_bag'))))
                        result['vo_cache'][variant]['status']='executed_shared'
                result['pass1']=str(group.relative_to(output))
            for variant,error in vo_errors.items():
                vf=vo_folders.get(variant) or folder_for(target/'vio_replay'/variant,'unavailable',native_ready)
                failure=vf/'failure_plot'
                if not args.plan_only:
                    health=load(vf/'raw_bag',('/ov_msckf/diagnostics/health',)) if (vf/'raw_bag/metadata.yaml').exists() else {}
                    failure_notes=figure_notes(health,context=('VO REPLAY FAILED: '+variant,'See summary.json and OpenVINS logs.'))
                    draw(failure,gps,{k:np.empty((0,13)) for k in ('vio','local','global')},background,'VO unavailable / '+variant,failure_notes)
                result['jobs'].append(dict(stage='vio',vio_variant=variant,status='failed',reason=error,failure_plot=str(failure.relative_to(output))))
                print('  [VO='+variant+'] 失敗: '+error+' / wheelは継続',flush=True)
            # hub作成は待ち時間のないファイル合成。全VOを確定してからEKFを動かす。
            hub_key=cache.digest(dict(common=cache.record_digest(lowband,COMMON_TOPICS),
                vo={v:cache.record_digest(p,[RAW_TOPIC]) if (p/'metadata.yaml').exists() else str(p) for v,p in available.items()},
                code=cache.implementation([backend[1]])))
            def hub_ready(p):
                try:return json.loads((p/'complete.json').read_text()).get('storage')==cache.source_identity(p/'input_bag')
                except (OSError,ValueError,KeyError):return False
            hub_dir=folder_for(target/'cache/input_hub',hub_key,hub_ready);hub=hub_dir/'input_bag'
            if not args.plan_only and not hub_ready(hub_dir):
                build_hub(bag,lowband,available,hub)
                (hub_dir/'complete.json').write_text(json.dumps(dict(storage=cache.source_identity(hub))))
            captured=snapshot.get('snapshots',{})
            historical_mode='legacy' if '/ekf_local_node' in captured else 'separated_offroad'
            if '/ekf_local_3d_node' in captured:
                vector=captured['/ekf_local_3d_node']['values'].get('imu0_config',[])
                historical_mode='ekf_3d_accel' if any(vector[12:15]) else 'ekf_3d_no_accel'
            if captured.get('/ekf_local_horizontal_node',{}).get('values',{}).get('base_link_frame')=='base_level':historical_mode='separated_projected'
            prepared=[];unique={};input_digests={}
            for variant in args.vio_variants:
                for magnetic in args.magnetic:
                    seeded=magnetic_seed_yaw(yaw, offset, magnetic)
                    schedule=[]
                    if variant in available:
                        if 'historical' in args.stages:
                            if snapshot:schedule.append(('historical',historical_mode,True,False))
                            else:result['jobs'].append(dict(stage='historical',vio_variant=variant,magnetic=magnetic,status='unavailable',reason='保存parameterなし'))
                        if 'comparison' in args.stages:schedule += [('comparison',m,False,False) for m in args.modes]
                        elif 'sources' in args.stages:schedule.append(('comparison','separated_offroad',False,False))
                    if 'sources' in args.stages:schedule += [('sources',m,False,True) for m in (args.modes if args.wheel_per_mode else ['separated_offroad'])]
                    for stage,mode,historical,wheel in schedule:
                        entry=dict(stage=stage,mode=mode,magnetic=magnetic,vio_variant=variant);result['jobs'].append(entry)
                        try:
                            with tempfile.TemporaryDirectory(prefix='plan_',dir=target) as temp:
                                temp=Path(temp);job=attach_hub(create_job(hub,temp,config,mode,seeded,snapshot,historical,wheel,args.duration,args.rate),hub,variant)
                                topics=tuple(job['input_topics'])
                                if topics not in input_digests:
                                    input_digests[topics]=cache.record_digest(hub,topics) if not args.plan_only else dict(planned=hub_key,topics=topics)
                                key=cache.ekf_key(job,temp,input_digests[topics],numerical_code,'all_conditions_twopass')
                                folder=folder_for(target/stage/('shared_wheel' if wheel else variant)/('mag_'+magnetic)/mode,key,cache.result_ready)
                                for path in temp.glob('*.yaml'):shutil.copy2(path,folder/path.name)
                            entry.update(path=str(folder.relative_to(output)),numerical_key=key)
                            job.update(vio_variant='not_used' if wheel else variant,heading=dict(result['heading'],magnetic=magnetic,seed_yaw_rad=seeded))
                            (folder/'job.json').write_text(json.dumps(job,ensure_ascii=False,indent=2))
                            prepared.append((entry,job,folder));unique[str(folder)]=(job,folder)
                        except Exception as exc:entry.update(status='failed',reason=str(exc))
            if len(unique)>args.max_ekf_graphs:raise ValueError('同時graph数%dが上限%dを超える。条件を絞るか上限を明示変更する'%(len(unique),args.max_ekf_graphs))
            status={};group_error=None
            result['distinct_ekf_graphs']=len(unique)
            if unique and not args.plan_only:
                waiting=sum(not(args.resume and cache.result_ready(p)) for _,p in unique.values())
                if waiting:
                    print('  PASS 2: 全条件 '+str(waiting)+' graph を共有再生',flush=True)
                    result['replays']['ekf']=1
                try:status=run_shared(list(unique.values()),folder_for(target/'pass2_ekf',hub_key),args)
                except Exception as exc:group_error=str(exc)
            # 方式別bagは従来のtopic名へ分離されるため、図・CSVの解釈は旧版と同じ。
            cached_inputs={};outputs={};processed={}
            for entry,job,folder in prepared:
                variant=entry['vio_variant'];magnetic=entry['magnetic'];mode=entry['mode'];wheel=job['wheel_only']
                try:
                    if args.plan_only:entry['status']='planned';continue
                    if str(folder) in processed:
                        entry['status']='reused_wheel' if wheel else processed[str(folder)]
                        print('  [VO='+variant+', magnetic='+magnetic+'] '+entry['stage']+' '+mode+': '+entry['status'],flush=True)
                        continue
                    entry['status']=status.get(str(folder),'failed')
                    if entry['status']=='failed':raise RuntimeError(group_error or '共有計算結果なし')
                    selected=load(folder/'output_bag',job['outputs'])
                    if not wheel and variant in available and not selected.get('/vio/odometry'):
                        raise RuntimeError('raw VOあり・adapter出力なし。TF/入力を確認')
                    transform,reason=placement(selected,gps)
                    if not transform:raise RuntimeError(reason)
                    series={name:mapped(selected.get(topic,[]),transform,frame) for name,topic,frame in
                        [('local','/odometry/local','odom'),('global','/odometry/global','map')]}
                    if any(not len(a) for a in series.values()):raise RuntimeError('local/global出力が空')
                    if variant not in cached_inputs:
                        inp={t:data.get(t,[]) for t in COMMON_TOPICS}
                        if variant in available and (available[variant]/'metadata.yaml').exists():
                            inp.update(load(available[variant],(RAW_TOPIC,'/ov_msckf/diagnostics/health')))
                        if args.duration:
                            start=min((ns for t in COMMON_TOPICS for ns,_ in inp.get(t,[])),default=0)
                            inp={t:[row for row in rows if row[0]<=start+int(args.duration*1e9)] for t,rows in inp.items()}
                        cached_inputs[variant]=inp
                    combined=dict(cached_inputs[variant]);combined.update(selected)
                    expected=[]
                    import re
                    for spec in job['nodes']:
                        if spec['executable']!='ekf_node':continue
                        for cfg in yaml.safe_load((folder/spec['parameters']).read_text()).values():
                            values=cfg.get('ros__parameters',{})
                            expected += [dict(spec['remaps']).get(v,v) for k,v in values.items() if re.fullmatch(r'(odom|imu|twist)\d+',k) and isinstance(v,str) and any(values.get(k+'_config',[True]))]
                    if wheel:combined.pop('/ov_msckf/diagnostics/health',None)
                    notes=figure_notes(combined,expected,('/odometry/local','/odometry/global'),(
                        'TWO PASS; VO='+variant+'; magnetic='+magnetic+'; mode='+mode+'; duration='+str(args.duration or 'full')+' s',
                        'VO intentionally excluded.' if wheel else 'Input observation does not guarantee successful fusion.'))
                    quality=output_quality(selected)
                    if quality['speed_warning']:notes.append('OUTPUT WARNING: estimated 3D speed exceeds 3 m/s.')
                    if quality['numerically_invalid']:notes.append('OUTPUT INVALID: covariance or velocity invalid.')
                    plot_key=cache.render_key(dict(key=entry['numerical_key'],storage=cache.source_identity(folder/'output_bag'),gnss=presentation),target/'basemap.npz' if background else None,render_code)
                    if cache.render_needed(folder,plot_key,['xy_full.png','z.png','states.png'],args.resume):
                        draw(folder,gps,series,background,mode+' / '+variant+' / magnetic '+magnetic,notes)
                        state_plots(selected,folder,notes);input_report(selected,folder);cache.mark_render(folder,plot_key)
                    (folder/'quality.json').write_text(json.dumps(quality,ensure_ascii=False,indent=2))
                    if quality['numerically_invalid']:raise RuntimeError('不正共分散/速度。quality.json参照')
                    if quality['speed_warning']:entry['status']='warning'
                    (folder/'complete.json').write_text(json.dumps(dict(status='complete',fingerprint=entry['numerical_key'],storage=cache.source_identity(folder/'output_bag'))))
                    outputs[str(folder)]=dict(local=series['local'],vio=mapped(selected.get('/vio/odometry',[]),transform,'odom'),notes=notes)
                    processed[str(folder)]=entry['status']
                except Exception as exc:
                    entry.update(status='failed',reason=str(exc));(folder/'complete.json').unlink(missing_ok=True)
                    if not (folder/'plot_annotations.json').exists():draw(folder,gps,{k:np.empty((0,13)) for k in ('local','global')},background,mode+' / unavailable',['COMPARISON FAILED: '+variant+' / '+mode])
                print('  [VO='+variant+', magnetic='+magnetic+'] '+entry['stage']+' '+mode+': '+entry['status'],flush=True)
            if not args.plan_only:
                for entry,job,folder in prepared:
                    if entry['stage']!='sources' or str(folder) not in outputs:continue
                    variant=entry['vio_variant'];magnetic=entry['magnetic']
                    candidates=[e for e,j,p in prepared if e['stage']=='comparison' and e['vio_variant']==variant and e['magnetic']==magnetic and str(p) in outputs]
                    candidates.sort(key=lambda e:e['mode']!='separated_offroad')
                    vo=(outputs[str(output/candidates[0]['path'])]['vio'] if candidates else np.empty((0,13)))
                    wheel=outputs[str(folder)]['local'];notes=['VO='+variant+'; wheel + IMU computed independently.']+outputs[str(folder)]['notes']
                    if not len(vo):notes.append('VO UNAVAILABLE: see failed VO/comparison condition.')
                    draw(folder/'source_comparison'/variant,gps,dict(vio=vo,wheel_imu=wheel),background,'VO / wheel + IMU',notes)
            result['status']='planned' if args.plan_only else ('partial_failure' if any(j.get('status') in ('failed','unavailable','warning') for j in result['jobs']) or map_status.startswith('地図取得失敗') else 'complete')
        except Exception as exc:
            result.update(status='failed',reason=str(exc));print('試行失敗: '+str(exc),flush=True)
        finally:
            # 1件の出力directoryや索引の書込失敗も、後続bagの解析を中断させない。
            # 結果保存失敗を成功扱いにはせず、最後の終了codeへ反映する。
            try:
                (target/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
            except OSError as exc:
                result.update(status='failed',report_error=str(exc))
                print('結果保存失敗: '+str(relative)+': '+str(exc),flush=True)
            try:
                update_index(output,all_results)
            except OSError as exc:
                result.update(status='failed',index_error=str(exc))
                print('索引保存失敗: '+str(relative)+': '+str(exc),flush=True)
    print('全bag候補の処理終了: '+str(len(all_results))+'件',flush=True)
    print('結果: '+str(output/'README.md'),flush=True)
    if any(r['status'] in ('failed','partial_failure') for r in all_results):raise SystemExit(1)


def update_index(output,results):
    """途中経過も読める索引。成功と未実行・失敗を区別する。"""
    (output/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    lines=['# rosbag自己位置推定・一括解析','',
           '記録結果と再計算結果は別ディレクトリです。再計算は初期yawの疑似初期化を使用します。',
           '磁気補正なしはoffset_degだけを除去し、磁気ベクトルのbias/scale校正は維持します。',
           'VOはraw OpenVINSから新adapterで再計算。GNSSは真値ではありません。','']
    for result in results:
        rel=result['relative'];lines += ['## '+rel,'',
            '- 状態: '+result.get('status','running'),
            '- [GNSS地図]('+rel+'/gnss/xy_full.png)',
            '- [記録local/global]('+rel+'/recorded/xy_gnss_area.png)',
            '- [入力・欠測]('+rel+'/input_topics.json)',
            '- [条件・失敗理由]('+rel+'/summary.json)','']
        if result.get('reason'):lines.append('- 失敗理由: '+result['reason'])
        for job in result['jobs']:
            if job.get('path'):
                p=job['path'];lines.append('- '+job.get('vio_variant','recorded')+' / '+job['stage']+' / '+job['mode']+' / mag_'+job['magnetic']+' / '+job['status']+
                    ': [XY]('+p+'/xy_gnss_area.png) · [全体]('+p+'/xy_full.png) · [z]('+p+'/z.png) · [状態]('+p+'/states.png) · [条件]('+p+'/job.json)')
                if job['stage']=='sources' and (output/p/'source_comparison'/job.get('vio_variant','recorded')/'xy_gnss_area.png').exists():
                    lines.append('  - [VO単独・wheel＋IMU比較]('+p+'/source_comparison/'+job.get('vio_variant','recorded')+'/xy_gnss_area.png) · [高さ]('+p+'/source_comparison/'+job.get('vio_variant','recorded')+'/z.png)')
            elif job.get('reason'):
                if job.get('failure_plot'):lines.append('- [欠落条件の注記付き図]('+job['failure_plot']+'/xy_full.png) · [z]('+job['failure_plot']+'/z.png)')
                lines.append('- '+job.get('vio_variant','')+' / '+job['stage']+' / '+job['status']+': '+job['reason'])
        lines.append('')
    (output/'README.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':main()
