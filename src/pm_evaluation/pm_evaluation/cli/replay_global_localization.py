"""センサbagを実機なしで現行local/global構成へ再生し、出力を保存する。"""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory


INPUTS = ['/wheel/odometry', '/wit/imu', '/wit/mag', '/vio/odometry',
          '/fix', '/navpvt']
OUTPUTS = ['/odometry/local', '/odometry/local_horizontal', '/odometry/local_vertical',
           '/odometry/global', '/odometry/global_horizontal', '/odometry/gps', '/odometry/gps/unverified',
           # GNSSがglobalに入る際のmap→odom座標変換と、両EKFの依存関係を検証する。
           '/tf',
           '/fix/gated', '/localization/initialization/status',
           '/vio/odometry/twist_gated', '/vio/twist_gate/diagnostics',
           '/vio/vertical_gate/diagnostics', '/gnss/fix_gate/diagnostics']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--local-ekf-config', type=Path,
                        help='比較用local EKF設定YAML。省略時は本番設定')
    parser.add_argument('--ros-domain-id', type=int, default=112)
    parser.add_argument('--navsat-odom-topic', default='/odometry/global',
                        choices=['/odometry/local','/odometry/global'],
                        help='navsat入力の比較用。既定は本番設定')
    parser.add_argument('--global-separated',dest='global_separated',action='store_true',
                        help='global水平EKFとlocalの高さ・姿勢を合成する（既定）')
    parser.add_argument('--global-3d',dest='global_separated',action='store_false',
                        help='比較用：従来のglobal 3D EKFに戻す')
    parser.set_defaults(global_separated=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit('既存出力を上書きしません: {}'.format(args.output_dir))
    if not args.bag.is_dir() or args.rate <= 0:
        raise SystemExit('bagと正の再生倍率を指定してください')
    args.output_dir.mkdir(parents=True)
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    configs = Path(get_package_share_directory('pm_config'))/'config'
    names = ['hwt905_mag_calibration.yaml', 'heading_initializer.yaml',
             'gnss_fix_gate.yaml', 'ekf_local_horizontal_vio_twist.yaml',
             'ekf_global_gnss_constrained.yaml', 'navsat_transform_heading_initialized.yaml']
    for name in names:
        shutil.copy2(configs/name, args.output_dir/name)
    if args.local_ekf_config:
        shutil.copy2(args.local_ekf_config, args.output_dir/'local_override.yaml')
    (args.output_dir/'replay_parameters.json').write_text(json.dumps(
        {'input_bag':str(args.bag), 'rate':args.rate, 'duration':args.duration,
         'topics':INPUTS, 'navsat_odom_topic':args.navsat_odom_topic,
         'global_separated':args.global_separated,
         'local_ekf_config':str(args.local_ekf_config) if args.local_ekf_config else None,
         'description':'元bagのVIOを再利用。センサ・走行制御は起動しない。'},
        ensure_ascii=False, indent=2), encoding='utf-8')
    processes = []
    files = []
    def start(command, log_name):
        stream = (args.output_dir/log_name).open('w', encoding='utf-8')
        files.append(stream)
        proc = subprocess.Popen(command, env=env, stdout=stream,
                                stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc
    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                # 正常終了を優先し、それでも反応しない場合はTERMで失敗を明示する。
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=10)
                raise RuntimeError('プロセスがSIGINTで停止しませんでした')
    try:
        launch_command = ['ros2','launch','pm_evaluation','replay_separated_global.launch.py',
                          'navsat_odom_topic:='+args.navsat_odom_topic,
                          'global_separated:='+str(args.global_separated).lower()]
        if args.local_ekf_config:
            launch_command.append('local_ekf_config:='+str(args.local_ekf_config.resolve()))
        launch = start(launch_command, 'launch.log')
        time.sleep(3)
        if launch.poll() is not None:
            raise RuntimeError('再生対象のlaunchが起動できません')
        recorder = start(['ros2','bag','record','--storage','mcap',
                          '-o',str(args.output_dir/'replayed_bag'),
                          *(INPUTS+OUTPUTS)], 'recorder.log')
        time.sleep(3)
        if recorder.poll() is not None:
            raise RuntimeError('結果recorderが起動できません')
        command = ['ros2','run','pm_evaluation','bag_clock_player',str(args.bag),
                   '--rate',str(args.rate),'--drain-seconds','2']
        if args.duration:
            command += ['--duration',str(args.duration)]
        for topic in INPUTS:
            command += ['--topic',topic]
        player = start(command, 'player.log')
        while player.poll() is None:
            if launch.poll() is not None or recorder.poll() is not None:
                raise RuntimeError('推定nodeまたはrecorderが再生中に停止しました')
            time.sleep(.5)
        if player.returncode != 0:
            raise RuntimeError('bag再生に失敗しました。player.logを確認してください')
        time.sleep(2)
        stop(recorder)
        processes.remove(recorder)
    finally:
        for proc in reversed(processes):
            stop(proc)
        for stream in files:
            stream.close()
    if not (args.output_dir/'replayed_bag'/'metadata.yaml').is_file():
        raise RuntimeError('結果bagのmetadata.yamlがありません')
    subprocess.check_call(['ros2','bag','info',str(args.output_dir/'replayed_bag')], env=env)


if __name__ == '__main__':
    main()
