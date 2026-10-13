#!/usr/bin/env python3
"""指定PIDの待ち状態・I/OとNVMe/dirty pageを読み取り、正常終了時に保存する。

対象processへsignalや制御commandは送らない。Ctrl-Cはプローブ自身を止めるだけ。
短いbag停滞とCPU待ち・書き込み待ちの時刻を照合するための診断である。
"""
import argparse
import json
import math
import os
import signal
import time
from pathlib import Path


def thread_snapshot(proc, syscalls=False):
    """mainだけでなくDDS/workerの待ち先・実行/待機累積nsを読み取る。"""
    result = {}
    for task in (proc/'task').iterdir():
        if not task.name.isdigit():
            continue
        try:
            result[task.name] = {
                'name': (task/'comm').read_text().strip(),
                'wchan': (task/'wchan').read_text().strip(),
                'schedstat': (task/'schedstat').read_text().strip(),
            }
            # syscall読取りはkernelの状態確認を伴い、全DDS threadを高頻度に
            # 調べると診断自体の負荷が増える。既定OFF、必要時もwriterだけに限定。
            if not syscalls or result[task.name]['name'] != 'pm_bag_writer':
                continue
            # wchanのrwsemだけではfilesystem lockとmemory mapping lockを
            # 区別できない。権限が許す範囲でsyscall番号・引数を読み、writeの
            # fd候補も保存する。対象threadへattach/signalは一切行わない。
            try:
                syscall = (task/'syscall').read_text().strip()
                result[task.name]['syscall'] = syscall
                fields = syscall.split()
                if len(fields) > 1 and fields[0] != 'running':
                    # 第1引数がfdであるとは限らない。候補として記録し、syscall
                    # 番号とarchに照らして解釈する（Jetson arm64のwriteは64）。
                    candidate = int(fields[1], 0)
                    if 0 <= candidate < 65536:
                        try:
                            result[task.name]['first_argument_fd_target'] = os.readlink(
                                proc/'fd'/str(candidate))
                        except OSError:
                            pass
            except (OSError, ValueError) as error:
                result[task.name]['syscall_error'] = str(error)
        except OSError as error:
            # 終了途中・permission不足も記録する。空白をCPU待ちなしと解釈しない。
            result[task.name] = {'error': str(error)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', required=True, type=int)
    parser.add_argument('--output', required=True)
    parser.add_argument('--interval', type=float, default=0.1, help='読み取り間隔[秒]')
    parser.add_argument('--max-samples', type=int, default=20000)
    parser.add_argument('--threads', action='store_true',
                        help='全threadの待機・schedstatも保存する。診断負荷が増える')
    parser.add_argument('--syscalls', action='store_true',
                        help='--threads時、writerだけのsyscall/fdも読む。既定OFF')
    args = parser.parse_args()
    path = Path(args.output).expanduser()
    if args.pid <= 0 or not math.isfinite(args.interval) or args.interval <= 0 or args.max_samples <= 0:
        parser.error('pid、interval、max-samplesは正数で指定してください')
    if path.exists():
        parser.error('既存の診断結果は上書きしません')
    running = True
    rows = []

    def finish(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)
    proc = Path('/proc') / str(args.pid)
    # PID再利用を追いかけず、指定時点のprocessの開始tickだけを観測対象とする。
    try:
        start_tick = (proc/'stat').read_text().rpartition(') ')[2].split()[19]
    except (OSError, IndexError):
        parser.error('指定processの開始時刻を取得できません')
    while running and proc.exists() and len(rows) < args.max_samples:
        row = {'wall_s': time.time(), 'monotonic_s': time.monotonic()}
        try:
            if (proc/'stat').read_text().rpartition(') ')[2].split()[19] != start_tick:
                break
            # wchanはmain threadの待ち先、schedstatは実行時間・runqueue待ち時間[ns]。
            # ioはprocessの累積値、diskstats/dirtyはsystem全体であり原因を単独断定しない。
            row['wchan'] = (proc / 'wchan').read_text().strip()
            row['io'] = (proc / 'io').read_text()
            row['schedstat'] = (proc / 'schedstat').read_text().strip()
            if args.threads:
                row['threads'] = thread_snapshot(proc, syscalls=args.syscalls)
            row['diskstats'] = '\n'.join(
                line for line in Path('/proc/diskstats').read_text().splitlines()
                if line.split()[2] == 'nvme0n1')
            row['dirty'] = '\n'.join(
                line for line in Path('/proc/meminfo').read_text().splitlines()
                if line.startswith(('Dirty:', 'Writeback:')))
        except (OSError, ValueError, IndexError) as error:
            # kernel設定・権限・対象終了で読めない場合を記録し、0と誤認しない。
            row['error'] = str(error)
        rows.append(row)
        time.sleep(args.interval)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rows) + '\n')
    print('RECORDER_WAIT_SAVED', path, 'count=', len(rows),
          'limit_reached=', len(rows) >= args.max_samples, flush=True)


if __name__ == '__main__':
    main()
