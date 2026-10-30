"""bag内のlocal/global相対zとGNSS高度を時間に沿って比較する。"""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages
import numpy as np

from pm_evaluation.cli.plot_bag_trajectories import GNSS_STATE_STYLE, find_mcap_files
from pm_evaluation.cli.plot_gnss_bags import nearest_state
from pm_evaluation.cli.plot_bag_trajectories import infer_rtk_state_from_msg


def read_bag(path):
    """header時刻でodometryと高度を並べ、NAV-PVT状態だけbag時刻で照合する。"""
    topics=['/odometry/local','/odometry/global','/fix','/navpvt']
    out={'local':[],'global':[],'fix':[],'states':[]}
    for file in find_mcap_files(path):
        for record in read_ros2_messages(file,topics=topics,log_time_order=True):
            m=record.ros_msg
            if record.channel.topic=='/navpvt':
                out['states'].append((int(record.log_time_ns),infer_rtk_state_from_msg(m)))
                continue
            t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if record.channel.topic=='/fix':
                if m.status.status>=0 and math.isfinite(m.altitude):
                    out['fix'].append((t,float(m.altitude),int(record.log_time_ns)))
            else:
                z=m.pose.pose.position.z
                if math.isfinite(z): out[record.channel.topic.rsplit('/',1)[-1]].append((t,float(z)))
    for key in out: out[key].sort(key=lambda v:v[0])
    return out


def plot_bag(bag, overwrite=False, output_dir=None):
    # 高さの出力先をbag本体から独立させる。
    output_dir=bag if output_dir is None else output_dir
    output_dir.mkdir(parents=True,exist_ok=True)
    data=read_bag(bag)
    if not all(data[k] for k in ('local','global','fix')):
        raise RuntimeError('local/globalまたは有効なGNSS高度がありません')
    local=np.asarray(data['local']); global_=np.asarray(data['global'])
    # 最初に共通して観測できた時点におけるlocal zとGNSS高度を一致させる。
    # 高度基準の異なる値を比較するための表示操作であり、真値校正ではない。
    fixes=[f for f in data['fix'] if local[0,0]<=f[0]<=local[-1,0]]
    if not fixes: raise RuntimeError('localとGNSS高度に共通時間がありません')
    anchor=fixes[0]
    index=int(np.argmin(abs(local[:,0]-anchor[0])))
    if abs(local[index,0]-anchor[0])>.5:
        raise RuntimeError('基準GNSS時刻近傍のlocal zがありません')
    offset=local[index,1]-anchor[1]
    t0=min(local[0,0],global_[0,0],fixes[0][0])
    xfix=np.array([f[0]-t0 for f in fixes]); alt=np.array([f[1] for f in fixes]); aligned=alt+offset
    states=[nearest_state(data['states'],f[2]) for f in fixes]
    fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
    axes[0].plot(local[:,0]-t0,local[:,1],label='Local z (recorded)',color='tab:blue',lw=1.1)
    axes[0].plot(global_[:,0]-t0,global_[:,1],label='Global z (recorded)',color='tab:red',lw=.9,linestyle='--',alpha=.8)
    axes[0].plot(xfix,aligned,color='black',alpha=.3,lw=.8,label='GNSS altitude (initial z aligned)')
    for state,style in GNSS_STATE_STYLE.items():
        indices=[i for i,v in enumerate(states) if v==state]
        if indices:
            axes[0].scatter(xfix[indices],aligned[indices],s=14,c=style['color'],marker=style['marker'],label=style['label'],zorder=3)
            axes[1].scatter(xfix[indices],alt[indices],s=14,c=style['color'],marker=style['marker'],zorder=3)
    axes[1].plot(xfix,alt,color='black',alpha=.35,lw=.8)
    axes[0].set(ylabel='relative z [m]',title=bag.name+' — recorded odometry / GNSS height')
    axes[1].set(xlabel='time from first message [s]',ylabel='GNSS altitude [m]',title='Raw GNSS altitude (absolute reference)')
    axes[0].legend(loc='best',fontsize=8,ncol=2)
    for axis in axes: axis.grid(alpha=.3)
    fig.tight_layout()
    output=output_dir/'recorded_height_gnss.png'
    if output.exists() and not overwrite: raise RuntimeError('既存ファイルを上書きしません: {}'.format(output))
    fig.savefig(output,dpi=160);plt.close(fig)
    # GNSSとlocalの同時刻差は表示基準で初期値を揃えた参考値。
    differences=[]
    for f in fixes:
        i=int(np.searchsorted(local[:,0],f[0]))
        if i==0 or i>=len(local) or local[i,0]-local[i-1,0]>1.: continue
        differences.append(float(np.interp(f[0],local[:,0],local[:,1])-(f[1]+offset)))
    with (output_dir/'recorded_height_gnss.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['stamp','gnss_altitude_m','gnss_aligned_z_m','navpvt_state'])
        w.writerows([(fix[0],fix[1],fix[1]+offset,state) for fix,state in zip(fixes,states)])
    return {'bag':bag.name,'status':'ok','local_count':len(local),'global_count':len(global_),
            'fix_count':len(fixes),'anchor_stamp':anchor[0],
            'anchor_local_z_m':float(local[index,1]),'anchor_gnss_altitude_m':anchor[1],
            'local_range_m':[float(local[:,1].min()),float(local[:,1].max())],
            'local_final_z_m':float(local[-1,1]),'global_final_z_m':float(global_[-1,1]),
            'gnss_raw_range_m':[float(alt.min()),float(alt.max())],
            'gnss_aligned_minus_local_median_abs_m':float(np.median(np.abs(differences))) if differences else None,
            'gnss_local_comparison_count':len(differences)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags',nargs='+',type=Path)
    parser.add_argument('--summary',required=True,type=Path)
    parser.add_argument('--overwrite',action='store_true',help='既存の画像・CSVを上書きする')
    parser.add_argument('--output-root',type=Path,help='解析結果の保存先。bag名のディレクトリを作成する')
    args=parser.parse_args();results=[]
    for path in args.bags:
        try: result=plot_bag(path,args.overwrite,args.output_root/path.name if args.output_root else None)
        except Exception as e: result={'bag':path.name,'status':'error','reason':str(e)}
        results.append(result);print(json.dumps(result,ensure_ascii=False),flush=True)
    args.summary.write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':
    main()
