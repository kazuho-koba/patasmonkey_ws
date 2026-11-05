"""metadata単位で記録図・履歴設定再現・５方式・VO/車輪IMU比較を一括実行する。"""
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

from pm_evaluation.localization_batch_io import (discover, metadata, read_snapshot,
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


def execute(job, folder, args, signature):
    """完了印はbag検証と描画が終わるまで書かない。再実行は異なるfingerprintへ。"""
    ready=folder/'complete.json'
    from pm_evaluation.localization_batch_cache import result_ready
    if args.resume and result_ready(folder):
        return 'cached'
    # 不完全出力を黙って上書きせず、試行ごとに別名へ退避する。
    if (folder/'output_bag').exists():
        attempt=folder/('incomplete_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        attempt.mkdir()
        shutil.move(str(folder/'output_bag'),str(attempt/'output_bag'))
    (folder/'job.json').write_text(json.dumps(job,ensure_ascii=False,indent=2)+'\n')
    env=os.environ.copy();env['ROS_DOMAIN_ID']=str(args.domain);env['ROS_LOCALHOST_ONLY']='1'
    env['PM_EVALUATION_MCAP_COMPRESSION']=args.mcap_compression
    worker_started=time.monotonic()
    with (folder/'worker.log').open('w') as stream:
        command=['ros2','run','pm_evaluation','batch_localization_worker',str(folder/'job.json')]
        worker=subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT)
        try:
            code=worker.wait()
        except BaseException:
            # worker自身が子のSIGINT終了をfinallyで実施するまで待つ。
            worker.send_signal(__import__('signal').SIGINT)
            worker.wait(timeout=180)
            raise
        if code:raise RuntimeError('再計算失敗: '+str(folder/'worker.log'))
    (folder/'execution.json').write_text(json.dumps({'mode':'sequential','worker_wall_sec':time.monotonic()-worker_started},indent=2))
    return 'executed'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_root',type=Path)
    parser.add_argument('--output-root',type=Path)
    parser.add_argument('--results-root',type=Path,help='workspace自動検出できないinstall環境用')
    parser.add_argument('--stages',nargs='+',choices=['recorded','historical','comparison','sources'],
                        default=['recorded','historical','comparison','sources'])
    parser.add_argument('--modes',nargs='+',choices=MODES,default=list(MODES))
    parser.add_argument('--magnetic',nargs='+',choices=['on','off'],default=['on','off'])
    parser.add_argument('--bag',action='append',help='bag相対パスまたはbasenameの絞り込み')
    parser.add_argument('--basemap',type=Path,help='既存OSM npzキャッシュ')
    parser.add_argument('--no-map',action='store_true',help='ネット接続なしで背景なしを明示')
    parser.add_argument('--duration',type=float,help='動作確認用の先頭bag秒')
    parser.add_argument('--rate',type=float,default=1.)
    parser.add_argument('--domain',type=int,default=191)
    parser.add_argument('--ekf-workers',type=int,default=5,choices=range(1,6),
                        help='１回の再生で並行実行する比較方式数。1は従来の逐次実行')
    parser.add_argument('--initial-yaw-deg',type=float,help='記録ENU yawがない場合の疑似初期方位')
    parser.add_argument('--vio-variants',nargs='+',type=lambda value: {'online_on':'online-cam-calib_on','online_off':'online-cam-calib_off'}.get(value,value),
                        choices=['recorded','online-cam-calib_on','online-cam-calib_off'],
                        default=['online-cam-calib_on','online-cam-calib_off'],
                        help='記録raw利用、OpenVINS外部校正ON再実行、OFF再実行')
    parser.add_argument('--vio-config',type=Path,help='保存初期校正がない場合のestimator YAML')
    parser.add_argument('--wheel-per-mode',action='store_true',help='VOを外した条件も各方式で計算')
    parser.add_argument('--plan-only',action='store_true',help='bag探索・入力検査・job生成まで')
    parser.add_argument('--no-lowband-cache',action='store_true',help='検証用：低帯域抽出を使わず元bagを読む')
    parser.add_argument('--keep-shared-bags',action='store_true',help='分離照合後も共有中間bagを保持する（既定は削除）')
    parser.add_argument('--mcap-compression',choices=['zstd','none'],default='zstd',help='ROS recorderのMCAP chunk圧縮。Python生成bagは従来どおりZSTD')
    parser.add_argument('--resume',dest='resume',action='store_true',default=True)
    parser.add_argument('--no-resume',dest='resume',action='store_false')
    args=parser.parse_args()
    if not args.input_root.is_dir():parser.error('入力ディレクトリなし')
    if not math.isfinite(args.rate) or args.rate<=0:parser.error('rateは有限正数')
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration<=0):parser.error('durationは有限正数')
    if not 1<=args.domain<=232:parser.error('domainは1〜232の隔離用番号')
    if args.initial_yaw_deg is not None and not math.isfinite(args.initial_yaw_deg):parser.error('初期yawは有限値')
    root=args.input_root.resolve()
    if args.results_root:results=args.results_root.resolve()
    else:
        workspace=next((p for p in [Path.cwd(),*Path.cwd().parents] if (p/'src/pm_evaluation').is_dir()),None)
        if workspace is None:parser.error('workspace内で実行するか--results-rootを指定')
        results=workspace/'src/pm_evaluation/results'
    output=(args.output_root or results/(root.name+'_batch')).resolve()
    if output==root or output in root.parents:parser.error('出力で入力階層を覆うことはできない')
    output.mkdir(parents=True,exist_ok=True)
    bags=discover(root,excluded=[output])
    if args.bag:bags=[p for p in bags if p.name in args.bag or str(p.relative_to(root)) in args.bag]
    if not bags:parser.error('解析対象metadataがない')
    config=Path(get_package_share_directory('pm_config'))/'config'
    code=list(Path(__file__).resolve().parent.parent.glob('localization_batch*.py'))+[Path(__file__).resolve(),Path(__file__).with_name('batch_localization_worker.py'),
          Path(__file__).with_name('batch_localization_group_worker.py'),
          Path(__file__).with_name('batch_vio_worker.py'),
          Path(__file__).resolve().parent.parent/'localization_vio_replay.py']
    # 共通transformやゲートが変わった場合も旧cacheを流用しない。
    import pm_localization
    code+=[Path(__file__).resolve().parent.parent/'localization_heading.py']
    code+=list(Path(pm_localization.__file__).parent.glob('*.py'))+list(config.glob('*.yaml'))
    if any(v!='recorded' for v in args.vio_variants):
        from ament_index_python.packages import get_package_prefix
        native=Path(get_package_prefix('ov_msckf'))/'lib'
        code += [native/'ov_msckf/run_subscribe_msckf']+list(native.glob('libov*.so'))
    options={k:v for k,v in vars(args).items() if k not in ('resume','plan_only','output_root')}
    options={k:str(v) if isinstance(v,Path) else v for k,v in options.items()}
    from pm_evaluation import localization_batch_cache as cache
    stage_code=cache.stage_code_files()
    native_vo=[]
    if any(v!='recorded' for v in args.vio_variants):
        native_vo=[native/'ov_msckf/run_subscribe_msckf']+list(native.glob('libov*.so'))
    all_results=[]
    for bag in bags:
        relative=bag.relative_to(root) if bag!=root else Path(bag.name)
        target=output/relative;target.mkdir(parents=True,exist_ok=True)
        result={'bag':str(bag),'relative':str(relative),'jobs':[]};all_results.append(result)
        print('解析開始: '+str(relative),flush=True)
        try:
            lowband=bag
            if not args.no_lowband_cache and not args.plan_only:
                lowband,extraction=cache.extract_lowband(bag,target/'cache/lowband')
                result['lowband_cache']=extraction
            data=load(lowband);gps,origin=fixes(data)
            input_digests={}
            def selected_identity(source,topics):
                identity=(str(source),tuple(topics))
                if identity not in input_digests:input_digests[identity]=cache.record_digest(source,topics)
                return input_digests[identity]
            presentation=cache.render_key(selected_identity(lowband,TOPICS),target/'basemap.npz',stage_code['render'])
            report=input_report(data,target)
            (target/'available_topics.json').write_text(json.dumps(metadata(bag)[0],ensure_ascii=False,indent=2))
            if args.plan_only:background=None;map_status='plan-only'
            else:background,map_status=map_cache(gps,origin,target/'basemap.npz',args.basemap,args.no_map)
            result['map']=map_status
            if not args.plan_only:
                presentation=cache.render_key(selected_identity(lowband,TOPICS),target/'basemap.npz' if background else None,stage_code['render'])
                (target/'gnss').mkdir(exist_ok=True)
                if cache.render_needed(target/'gnss',presentation,['xy_full.png','z.png'],args.resume):
                    draw(target/'gnss',gps,{},background,bag.name+' GNSS')
                    cache.mark_render(target/'gnss',presentation)
                if gps:
                    shutil.copy2(target/'gnss/xy_full.png',bag/'gnss_trajectory_osm.png')
                    shutil.copy2(target/'gnss/xy_full.png',target/'gnss_trajectory_osm.png')
                if 'recorded' in args.stages:
                    folder=target/'recorded';folder.mkdir(exist_ok=True)
                    if cache.render_needed(folder,presentation,['xy_full.png','z.png','states.png'],args.resume):
                        render_recorded(data,gps,background,folder,read_snapshot(bag))
                        recorded_notes=json.loads((folder/'plot_annotations.json').read_text())
                        state_plots(data,folder,recorded_notes)
                        cache.mark_render(folder,presentation)
            if set(args.stages)=={'recorded'}:
                result['status']='planned' if args.plan_only else ('partial_failure' if map_status.startswith('地図取得失敗') else 'complete')
                continue
            # 旧adapter出力から正しい全共分散を逆算することはできない。
            # raw不在を黙って旧/vioで代用せず、その試行の再計算を失敗として記録する。
            for required in ('/wheel/odometry','/wit/imu','/tf_static'):
                if not data.get(required):raise ValueError('必要入力なし: '+required)
            snapshot=read_snapshot(bag)
            (target/'recorded_parameters.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2))
            yaw,offset,method=heading(data,snapshot,config,
                      None if args.initial_yaw_deg is None else math.radians(args.initial_yaw_deg))
            result['heading']=dict(method,recorded_yaw_rad=yaw,offset_deg=offset)
            local_code=code+list((bag/'provenance').glob('*.json'))+list((bag/'provenance/configuration').rglob('*.yaml'))
            if args.vio_config:local_code+=list(args.vio_config.parent.glob('*.yaml'))
            signature=fingerprint(bag,options,local_code)[:16]
            (target/'implementation.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in local_code},indent=2))
            captured=snapshot.get('snapshots',{})
            historical_mode=('legacy' if '/ekf_local_node' in captured else
                             'ekf_3d_no_accel' if '/ekf_local_3d_node' in captured else 'separated_offroad')
            # 当時の3D accel選択はsnapshotの融合vectorから判別する。
            if '/ekf_local_3d_node' in captured:
                vector=captured['/ekf_local_3d_node']['values'].get('imu0_config',[])
                historical_mode='ekf_3d_accel' if any(vector[12:15]) else 'ekf_3d_no_accel'
            if '/ekf_local_horizontal_node' in captured:
                vals=captured['/ekf_local_horizontal_node']['values']
                if vals.get('base_link_frame')=='base_level':historical_mode='separated_projected'
            from pm_evaluation.localization_vio_replay import configuration, configuration_files, merge_input, SENSOR_TOPICS
            run_stamp=cache.analysis_timestamp().strftime('%Y%m%d_%H%M')
            analysis_paths={}
            def analysis_folder(parent,key,ready=None):
                # 同一実行内のwheel共有は--no-resumeでも維持し、別実行は日時で区別する。
                lookup=(str(parent),key)
                if lookup not in analysis_paths:
                    analysis_paths[lookup]=cache.analysis_directory(parent,key,run_stamp,args.resume,ready)
                return analysis_paths[lookup]
            computed_wheels=set()
            for variant in args.vio_variants:
                # 校正条件を先に表示し、時間のかかるVO再実行中も処理中の条件を識別する。
                print('  VO条件: '+variant,flush=True)
                replay_bag=lowband
                vo_failed=False
                if variant=='recorded':
                    if not data.get('/ov_msckf/odomimu'):
                        missing=dict(stage='vio',vio_variant=variant,status='unavailable',reason='記録raw VIOなし')
                        result['jobs'].append(missing)
                        if not args.plan_only:
                            failure=analysis_folder(target/'vio_replay'/variant,'recorded_raw_missing',cache.vo_ready)/'failure_plot'
                            draw(failure,gps,{k:np.empty((0,13)) for k in ('vio','local','global')},background,
                                 'Unavailable comparison / '+variant,['RECORDED RAW VO MISSING: /ov_msckf/odomimu'])
                            missing['failure_plot']=str(failure.relative_to(output))
                        vo_failed=True
                else:
                    # 校正ファイル不足もVO条件の失敗に留め、wheelの独立計算を止めない。
                    vf=None
                    try:
                        settings=configuration_files(bag,args.vio_config)
                        vo_signature=cache.vio_key(cache.source_identity(bag),variant,args.rate,args.duration,settings,stage_code['vo']+native_vo)[:20]
                        vf=analysis_folder(target/'vio_replay'/variant,vo_signature,cache.vo_ready)
                        counts={x['topic_metadata']['name']:x['message_count'] for x in metadata(bag)[0]['topics_with_message_count']}
                        if not all(counts.get(t,0)>0 for t in SENSOR_TOPICS):
                            raise ValueError('OpenVINS再実行に必要な左右画像またはOAK IMUなし')
                        cfg=configuration(bag,vf/'config',variant,args.vio_config)
                        spec=dict(bag=str(bag),config=str(cfg),rate=args.rate,duration=args.duration)
                        (vf/'job.json').write_text(json.dumps(spec,indent=2))
                        replay_bag=vf/'merged_input'
                        vo_hit=args.resume and cache.vo_ready(vf)
                        result.setdefault('vo_cache',{})[variant]=dict(key=vo_signature,status='cached' if vo_hit else 'recomputed')
                        if not args.plan_only and not vo_hit:
                            for name in ('raw_bag','merged_input'):
                                if (vf/name).exists():
                                    shutil.move(str(vf/name),str(vf/(name+'_incomplete_'+datetime.now().strftime('%Y%m%d_%H%M%S'))))
                            env=os.environ.copy();env['ROS_DOMAIN_ID']=str(args.domain);env['ROS_LOCALHOST_ONLY']='1'
                            with (vf/'worker.log').open('w') as stream:
                                process=subprocess.Popen(['ros2','run','pm_evaluation','batch_vio_worker',str(vf/'job.json')],env=env,stdout=stream,stderr=subprocess.STDOUT)
                                try:returncode=process.wait()
                                except BaseException:
                                    process.send_signal(__import__('signal').SIGINT);process.wait(timeout=90);raise
                                if returncode:raise RuntimeError('OpenVINS再実行失敗: '+str(vf/'worker.log'))
                            merge_input(bag,vf/'raw_bag',replay_bag,input_records=lowband)
                            (vf/'complete.json').write_text(json.dumps({'status':'complete','fingerprint':vo_signature,'storage':cache.source_identity(replay_bag)}))
                    except Exception as exc:
                        if vf is None:vf=analysis_folder(target/'vio_replay'/variant,'configuration_failure',cache.vo_ready)
                        result['jobs'].append(dict(stage='vio',vio_variant=variant,status='failed',reason=str(exc)))
                        print('  [VO='+variant+'] VO条件失敗: '+str(exc),flush=True)
                        if not args.plan_only:
                            # 失敗した条件もGNSSだけの代替図を残し、比較線の欠落を隠さない。
                            failure=vf/'failure_plot'
                            diagnostic=load(vf/'raw_bag',('/ov_msckf/diagnostics/health',)) if (vf/'raw_bag/metadata.yaml').exists() else {}
                            notes=figure_notes(diagnostic,context=(
                                'VO REPLAY FAILED: '+variant,
                                'Expected VO / local / global trajectories unavailable; see worker.log.',))
                            draw(failure,gps,{k:np.empty((0,13)) for k in ('vio','local','global')},background,
                                 'Unavailable comparison / '+variant,notes)
                            result['jobs'][-1]['failure_plot']=str(failure.relative_to(output))

                        vo_failed=True
                        replay_bag=lowband
                # 元センサとVO healthは出力bagに含まれないため、再生入力から別に読む。
                annotation_inputs=load(replay_bag,('/wheel/odometry','/wit/imu','/fix','/ov_msckf/odomimu','/ov_msckf/diagnostics/health')) if not args.plan_only else {}
                if args.duration and annotation_inputs:
                    # 区間外のVO異常を短時間比較の入力異常として注記しない。
                    start=min(ns for rows in annotation_inputs.values() for ns,_ in rows)
                    end=start+int(args.duration*1e9)
                    annotation_inputs={t:[r for r in rows if r[0]<=end] for t,rows in annotation_inputs.items()}
                for magnetic in args.magnetic:
                    # yaw_ENU=pi/2-(bearing+offset)。offsetを除くとyawはoffsetだけ変化する。
                    seeded_yaw=magnetic_seed_yaw(yaw, offset, magnetic)
                    schedule=[]
                    if 'historical' in args.stages:
                        if snapshot:schedule.append(('historical',historical_mode,True,False))
                        else:result['jobs'].append({'stage':'historical','magnetic':magnetic,'status':'unavailable','reason':'当時parameterなし'})
                    if 'comparison' in args.stages:schedule += [('comparison',m,False,False) for m in args.modes]
                    if 'sources' in args.stages:
                        schedule += [('sources',m,False,True) for m in (args.modes if args.wheel_per_mode else ['separated_offroad'])]
                        # source-onlyでも新adapterの独立軌跡を得るため１本を実行する。
                        if 'comparison' not in args.stages:schedule.append(('comparison','separated_offroad',False,False))
                    if vo_failed:
                        # VOを必要とする比較は実行しない。wheelは元bagの独立入力だけで継続する。
                        schedule=[item for item in schedule if item[3]]
                    comparisons={};wheel_outputs=[];prepared=[]
                    for stage,mode,historical,wheel in schedule:
                        folder=target/stage/variant/('mag_'+magnetic)/mode/run_stamp
                        entry={'stage':stage,'mode':mode,'magnetic':magnetic,'vio_variant':variant,'path':str(folder.relative_to(output))}
                        result['jobs'].append(entry)
                        try:
                            source=lowband if wheel else replay_bag
                            with tempfile.TemporaryDirectory(prefix='plan_',dir=target) as planning:
                                planning=Path(planning)
                                job=create_job(source,planning,config,mode,seeded_yaw,snapshot,historical,wheel,args.duration,args.rate)
                                model_input=(selected_identity(source,job['input_topics']) if (source/'metadata.yaml').exists() else
                                    dict(planned_vo=variant,key=vo_signature,topics=job['input_topics']))
                                numerical=cache.ekf_key(job,planning,model_input,
                                      stage_code['ekf'],args.ekf_workers if stage=='comparison' else 1)[:20]
                                folder=analysis_folder(target/stage/('shared_wheel' if wheel else variant)/('mag_'+magnetic)/mode,numerical,cache.result_ready)
                                folder.mkdir(parents=True,exist_ok=True)
                                for param in planning.glob('*.yaml'):shutil.copy2(param,folder/param.name)
                            entry['path']=str(folder.relative_to(output));entry['numerical_key']=numerical
                            job['vio_variant']='not_used' if wheel else variant
                            job['heading']=dict(result['heading'],magnetic=magnetic,seed_yaw_rad=seeded_yaw)
                            (folder/'job.json').write_text(json.dumps(job,ensure_ascii=False,indent=2)+'\n')
                            prepared.append((stage,mode,wheel,folder,entry,job))
                        except Exception as exc:
                            entry.update(status='failed',reason=str(exc))
                    # 比較５方式だけを共通再生する。歴史設定とwheel比較は独立の
                    # 逐次実行を残し、入力区間や状態の前提が異なるものを混ぜない。
                    shared_status={};shared_errors={}
                    if not args.plan_only and args.ekf_workers>1:
                        from pm_evaluation.localization_batch_parallel import run_shared
                        candidates=[(j,f) for st,m,w,f,e,j in prepared if st=='comparison']
                        for begin in range(0,len(candidates),args.ekf_workers):
                            group=candidates[begin:begin+args.ekf_workers]
                            try:
                                shared_status.update(run_shared(group,analysis_folder(target/'shared_replays'/variant/('mag_'+magnetic),signature),args))
                            except Exception as exc:
                                for _,f in group:shared_errors[str(f)]=str(exc)
                    for stage,mode,wheel,folder,entry,job in prepared:
                        output_plotted=False
                        try:
                            if args.plan_only:entry['status']='planned';continue
                            if str(folder) in shared_errors:raise RuntimeError(shared_errors[str(folder)])
                            if wheel and str(folder) in computed_wheels:entry['status']='reused_wheel'
                            elif str(folder) in shared_status:entry['status']=shared_status[str(folder)]
                            else:entry['status']=execute(job,folder,args,signature)
                            selected=load(folder/'output_bag',job['outputs'])
                            # 軌跡ファイルの存在だけで成功としない。配信rawがあるのに
                            # TF/接続不良でVIOが無視される比較を検出する。
                            delivery_path=folder/'delivery.json'
                            if (folder/'execution.json').exists():
                                execution=json.loads((folder/'execution.json').read_text())
                                if execution.get('mode')=='shared_replay':delivery_path=Path(execution['delivery'])
                            if (not job['wheel_only'] and delivery_path.exists() and
                                json.loads(delivery_path.read_text()).get('published',{}).get('/ov_msckf/odomimu',0)>0
                                and not selected.get('/vio/odometry')):
                                (folder/'complete.json').unlink(missing_ok=True)
                                raise RuntimeError('raw VO配信あり・adapter出力なし。TFと初期整列を確認')
                            transform,reason=placement(selected,gps)
                            if not transform:raise RuntimeError(reason)
                            series={name:mapped(selected.get(topic,[]),transform,frame) for name,topic,frame in
                                    [('local','/odometry/local','odom'),('global','/odometry/global','map')]}
                            if not len(series['local']) or not len(series['global']):raise RuntimeError('local/global出力が空')
                            # 実際の生成YAMLからEKFが購読する観測topicを抽出する。
                            expected=[]
                            import re
                            for spec in job['nodes']:
                                if spec['executable']!='ekf_node':continue
                                cfg=yaml.safe_load((folder/spec['parameters']).read_text())
                                for node_cfg in cfg.values():
                                    for key,value in node_cfg.get('ros__parameters',{}).items():
                                        if (re.fullmatch(r'(odom|imu|twist)\d+',key) and isinstance(value,str)
                                                and any(node_cfg.get('ros__parameters',{}).get(key+'_config',[True]))):
                                            expected.append(dict(spec['remaps']).get(value,value))
                            combined=dict(annotation_inputs);combined.update(selected)
                            if wheel:combined.pop('/ov_msckf/diagnostics/health',None)
                            notes=figure_notes(combined,sorted(set(expected)),('/odometry/local','/odometry/global'),
                                ('VO='+variant+'; magnetic='+magnetic+'; mode='+mode+'; duration='+str(args.duration or 'full')+' s',
                                 'VO intentionally excluded.' if wheel else 'Input observation does not guarantee successful fusion.'))
                            quality=output_quality(selected)
                            if quality['speed_warning']:notes.append('OUTPUT WARNING: estimated 3D speed exceeds 3 m/s.')
                            if quality['numerically_invalid']:notes.append('OUTPUT INVALID: covariance or velocity is non-finite/invalid.')
                            plot_key=cache.render_key(dict(ekf=entry['numerical_key'],storage=cache.source_identity(folder/'output_bag'),gnss=presentation),
                                target/'basemap.npz' if background else None,stage_code['render'])
                            if cache.render_needed(folder,plot_key,['xy_full.png','z.png','states.png'],args.resume):
                                draw(folder,gps,series,background,mode+' / magnetic '+magnetic,notes)
                                state_plots(selected,folder,notes);input_report(selected,folder)
                                cache.mark_render(folder,plot_key)
                            else:entry['render_status']='cached'
                            output_plotted=True
                            (folder/'quality.json').write_text(json.dumps(quality,ensure_ascii=False,indent=2))
                            if quality['numerically_invalid']:
                                (folder/'complete.json').unlink(missing_ok=True)
                                raise RuntimeError('数値的に不正な出力共分散。quality.jsonを確認')
                            if quality['speed_warning']:entry['status']='warning'
                            (folder/'complete.json').write_text(json.dumps({'fingerprint':entry['numerical_key'],'status':'complete','storage':cache.source_identity(folder/'output_bag')},indent=2))
                            if wheel:
                                computed_wheels.add(str(folder))
                                wheel_outputs.append((mode,folder,selected,transform))
                            elif stage=='comparison':comparisons[mode]=(folder,selected,transform)
                        except Exception as exc:
                            entry['status']='failed';entry['reason']=str(exc)
                            # 位置合わせ不能・出力0件でも、欠落した比較条件の図を残す。
                            if not args.plan_only and not output_plotted:
                                failed_notes=['COMPARISON FAILED: '+variant+' / '+mode,
                                    'No valid mapped result; inspect summary.json and worker.log.']
                                draw(folder,gps,{k:np.empty((0,13)) for k in ('local','global')},background,
                                     mode+' / unavailable',failed_notes)
                            print('  [VO='+variant+', magnetic='+magnetic+'] '+stage+' '+mode+' 失敗: '+str(exc),flush=True)
                        print('  [VO='+variant+', magnetic='+magnetic+'] '+stage+' '+mode+': '+entry['status'],flush=True)
                    if not args.plan_only and wheel_outputs:
                        vo_folder=None;selected={};vio=np.empty((0,13))
                        if comparisons:
                            vo_folder,selected,transform=comparisons.get('separated_offroad',next(iter(comparisons.values())))
                            vio=mapped(selected.get('/vio/odometry',[]),transform,'odom')
                        for mode,folder,wheel_data,wheel_transform in wheel_outputs:
                            wheel=mapped(wheel_data.get('/odometry/local',[]),wheel_transform,'odom')
                            # 全系列に同じmap原点を使用し、各々をGNSSへfitしない。
                            comparison_folder=folder/'source_comparison'/variant
                            comparison_folder.mkdir(parents=True,exist_ok=True)
                            source_plot_key=cache.render_key(dict(vo=cache.source_identity(vo_folder/'output_bag') if vo_folder else 'unavailable',
                                wheel=cache.source_identity(folder/'output_bag'),gnss=presentation),
                                target/'basemap.npz' if background else None,stage_code['render'])
                            if cache.render_needed(comparison_folder,source_plot_key,['xy_full.png','z.png'],args.resume):
                                source_notes=figure_notes(dict(annotation_inputs,**selected),('/ov_msckf/odomimu','/vio/odometry'),('/fix',),('VO='+variant+'; wheel + IMU excludes VO.; duration='+str(args.duration or 'full')+' s',))
                                if not comparisons:
                                    source_notes.append('VO UNAVAILABLE: replay failed or no valid comparison output; wheel + IMU computed independently.')
                                draw(comparison_folder,gps,{'vio':vio,'wheel_imu':wheel},background,'New adapter VO / wheel + IMU',source_notes)
                                cache.mark_render(comparison_folder,source_plot_key)

            result['status']='planned' if args.plan_only else ('partial_failure' if any(j['status'] in ('failed','unavailable','warning') for j in result['jobs']) or map_status.startswith('地図取得失敗') else 'complete')
        except Exception as exc:
            result['status']='failed';result['reason']=str(exc)
            print('試行失敗: '+str(exc),flush=True)
        finally:
            (target/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            update_index(output,all_results)
    print('結果: '+str(output/'README.md'),flush=True)
    if any(r.get('status') in ('failed','partial_failure') for r in all_results):raise SystemExit(1)


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
