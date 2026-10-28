"""再生global EKFの位置・速度と車輪入力を同じ時間軸で表示する。"""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay_dir',type=Path)
    args=parser.parse_args()
    root=args.replay_dir
    audit=json.loads((root/'global_audit.json').read_text())
    start=audit['ready_global_header_sec']
    def read(name):
        with (root/name).open() as stream:
            return list(csv.DictReader(stream))
    global_=read('odometry_global_audit.csv')
    wheel=read('wheel_odometry_audit.csv')
    global_=[r for r in global_ if float(r['t'])>=start]
    wheel=[r for r in wheel if float(r['t'])>=start]
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    for name in ('x','y','z'):
        axes[0].plot([float(r['t'])-start for r in global_],
                     [float(r[name]) for r in global_],label=name)
    for name in ('vx','vy','vz'):
        axes[1].plot([float(r['t'])-start for r in global_],
                     [float(r[name]) for r in global_],label=name)
    axes[2].plot([float(r['t'])-start for r in wheel],
                 [float(r['vx']) for r in wheel],label='wheel vx')
    axes[0].set_ylabel('global position [m]')
    axes[1].set_ylabel('global velocity [m/s]')
    axes[2].set_ylabel('wheel vx [m/s]')
    axes[2].set_xlabel('time from initialization [s]')
    for ax in axes:ax.grid(alpha=.3);ax.legend()
    fig.tight_layout();fig.savefig(root/'global_state_audit.png',dpi=160)


if __name__=='__main__':
    main()
