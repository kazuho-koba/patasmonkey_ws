#!/usr/bin/env python3
"""駆動系を再生せず、同じ入力の再記録でrecorderの欠測を検査する。

Foxy→外部workspace→main workspaceをsourceした環境で実行する。
選択する入力はIMU/画像だけ。専用domainとlocalhost限定通信を使用し、
実機センサ・EKF・車両ノードは起動しない。結果は入力payloadのSHA256と
topicの多重集合で比較し、元bagに既に存在する欠測を新規欠測と誤認しない。
"""
import argparse
from collections import Counter
from contextlib import ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

from mcap.reader import make_reader


IMU_TOPICS = ['/oak/imu/data', '/wit/imu']
IMAGE_TOPICS = ['/oak/stereo/left/image_raw', '/oak/stereo/right/image_raw',
                '/oak/color/image_raw', '/oak/depth/image_raw']


def records(bag, topics):
    """分割MCAPを既存playerと同じ順序で読み、必要なpayloadだけを返す。"""
    for path in sorted(bag.glob('*.mcap')):
        with path.open('rb') as stream:
            for _, channel, message in make_reader(stream).iter_messages(topics=topics):
                yield message.log_time, channel.topic, message.data


def signature(topic, data):
    # header時刻だけでは重複messageを区別できないため、全payloadを照合する。
    return topic, hashlib.sha256(data).hexdigest()


def expected_messages(bag, topics, duration):
    """再生期間の境界はplayerと同じ、最初の選択messageからのbag秒数。"""
    counts = Counter()
    first = None
    for stamp, topic, data in records(bag, topics):
        if first is None:
            first = stamp
        if (stamp - first) * 1e-9 > duration:
            break
        counts[signature(topic, data)] += 1
    if first is None:
        raise ValueError('入力bagに選択topicがありません')
    return counts


def recorded_messages(bag, topics, storage):
    """出力もdeserializeせず比較する。SQLiteはread-onlyで開く。"""
    counts = Counter()
    if storage == 'mcap':
        for _, topic, data in records(bag, topics):
            counts[signature(topic, data)] += 1
    else:
        for path in sorted(bag.glob('*.db3')):
            with sqlite3.connect(path.as_uri() + '?mode=ro', uri=True) as connection:
                for topic, data in connection.execute(
                        'SELECT topics.name, messages.data FROM messages '
                        'JOIN topics ON topics.id=messages.topic_id ORDER BY messages.id'):
                    if topic in topics:
                        counts[signature(topic, data)] += 1
    return counts


def missing_examples(bag, missing, limit=20):
    """不足payloadの元bag時刻を示す。重複payloadでは欠測した出現位置を断定しない。"""
    topics = sorted({topic for topic, _ in missing})
    examples = []
    if not topics:
        return examples
    remaining = missing.copy()
    for stamp, topic, data in records(bag, topics):
        key = signature(topic, data)
        if remaining[key] > 0:
            examples.append(dict(topic=topic, source_log_time_ns=stamp,
                                 payload_sha256=key[1]))
            remaining[key] -= 1
            if len(examples) >= limit or not any(remaining.values()):
                break
    return examples


def stop(child):
    """自分が起動したprocess groupだけをSIGINTで終了し、制御sessionを維持する。"""
    if child is not None and child.poll() is None:
        try:
            os.killpg(child.pid, signal.SIGINT)
        except ProcessLookupError:
            # poll直後に自発終了した場合だけ許容し、別PIDへsignalを送らない。
            pass
        child.wait()  # 強制終了で未flushのbagを完成扱いしない。


def recording_groups(topics, separate_imu):
    """再生入力を変えず、記録プロセスの担当topicだけを排他的に分ける。"""
    if not separate_imu:
        return [('bag', topics)]
    return [('bag_imu', [topic for topic in topics if topic in IMU_TOPICS]),
            ('bag_images', [topic for topic in topics if topic in IMAGE_TOPICS])]


def stop_all(children):
    """終了処理の例外があっても全所有プロセスの停止を試み、最初の例外を戻す。"""
    first_error = None
    for child in children:
        try:
            stop(child)
        except BaseException as error:
            # Ctrl-Cの再入力も含めて他方のrecorderを放置しない。例外は握りつぶさない。
            if first_error is None:
                first_error = error
    if first_error is not None:
        raise first_error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=40.0)
    parser.add_argument('--mode', choices=['imu', 'images'], default='imu')
    parser.add_argument('--storage', choices=['mcap', 'sqlite3'], default='mcap')
    parser.add_argument('--domain', type=int, default=78)
    parser.add_argument('--qos-file', type=Path, default=None,
                        help='任意のrecorder QoS比較用YAML。省略時は自動QoS')
    parser.add_argument('--separate-imu', action='store_true',
                        help='images条件の記録だけをIMUと画像の2プロセスに分ける')
    args = parser.parse_args()
    def interrupt(signum, frame):
        # SIGTERMでもfinallyを経由し、所有するrecorderを未終了のまま残さない。
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('durationは有限の正の秒数で指定してください')
    if not 1 <= args.domain <= 101:
        parser.error('隔離domainは1～101で指定してください')
    if args.separate_imu and args.mode != 'images':
        parser.error('--separate-imuは--mode imagesとの組合せで使用してください')
    if args.qos_file is not None:
        args.qos_file = args.qos_file.resolve()
        if not args.qos_file.is_file():
            parser.error('QoSファイルが存在しません')
    args.bag = args.bag.resolve()
    args.output = args.output.resolve()
    if args.output.exists():
        parser.error('既存結果は上書きしません')
    topics = IMU_TOPICS + (IMAGE_TOPICS if args.mode == 'images' else [])
    # 事前に期待値を確定し、比較のhash処理で再生中のCPUを消費しない。
    expected = expected_messages(args.bag, topics, args.duration)
    args.output.mkdir(parents=True)
    if args.qos_file is not None:
        # 原本が後日変更されても、今回実際に読ませた設定を追跡できるようにする。
        (args.output / 'recorder_qos.yaml').write_text(args.qos_file.read_text())
    player_path = Path(__file__).resolve().parents[1] / 'pm_evaluation/cli/bag_clock_player.py'
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
    recorders = []
    player = None
    started = time.time()
    with ExitStack() as logs:
        player_log = logs.enter_context((args.output / 'player.log').open('w'))
        try:
            for name, group_topics in recording_groups(topics, args.separate_imu):
                bag = args.output / name
                log_name = 'record.log' if name == 'bag' else name + '_record.log'
                record_log = logs.enter_context((args.output / log_name).open('w'))
                command = ['ros2', 'bag', 'record', '-s', args.storage, '-o', str(bag),
                           '--max-cache-size', '0', '-b', '0']
                if args.qos_file is not None:
                    command += ['--qos-profile-overrides-path', str(args.output / 'recorder_qos.yaml')]
                command += group_topics
                recorder = subprocess.Popen(command, env=env, stdout=record_log,
                                            stderr=subprocess.STDOUT, start_new_session=True)
                recorders.append(dict(name=name, topics=group_topics, command=command,
                                      process=recorder))
            time.sleep(2.0)
            if any(item['process'].poll() is not None for item in recorders):
                raise RuntimeError('recorder起動失敗。record.logを確認してください')
            replay = ['python3', str(player_path), str(args.bag), '--duration', str(args.duration),
                      '--drain-seconds', '2']
            for topic in topics:
                replay += ['--topic', topic]
            player = subprocess.Popen(replay, env=env, stdout=player_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            player_started = time.monotonic()
            print('REPLAY_RECORD_READY', args.output, flush=True)
            while player.poll() is None:
                if any(item['process'].poll() is not None for item in recorders):
                    raise RuntimeError('再生中にrecorderが終了しました')
                time.sleep(0.2)
            if player.returncode != 0:
                raise RuntimeError('playerが異常終了しました')
            player_elapsed = time.monotonic() - player_started
            # publish完了後のDDS/recorder処理を待つ。待ち時間は全条件で同じ。
            time.sleep(3.0)
        finally:
            # 片方だけ停止して成功扱いしない。各bagのmetadata/終了コードを後で検査する。
            stop_all([player] + [item['process'] for item in reversed(recorders)])
    actual = Counter()
    recorder_results = []
    for item in recorders:
        bag = args.output / item['name']
        recorder = item['process']
        if not (bag / 'metadata.yaml').exists():
            raise RuntimeError('metadata.yamlがなく、bag未完了です: {}'.format(bag))
        # Foxy CLIは通常のSIGINTでも2を返すことがある。他のコードは成功扱いしない。
        if recorder.returncode not in (0, 2, -signal.SIGINT):
            raise RuntimeError('recorderの終了コードが異常です: {}'.format(recorder.returncode))
        info = subprocess.run(['ros2', 'bag', 'info', str(bag)], env=env,
                              capture_output=True, text=True, check=True)
        info_name = 'bag_info.txt' if item['name'] == 'bag' else item['name'] + '_info.txt'
        (args.output / info_name).write_text(info.stdout + info.stderr)
        actual.update(recorded_messages(bag, item['topics'], args.storage))
        recorder_results.append(dict(name=item['name'], topics=item['topics'],
                                     command=item['command'], exit=recorder.returncode))
    missing, extra = expected - actual, actual - expected
    rows = []
    for topic in topics:
        rows.append(dict(topic=topic,
                         expected=sum(n for (name, _), n in expected.items() if name == topic),
                         recorded=sum(n for (name, _), n in actual.items() if name == topic),
                         missing=sum(n for (name, _), n in missing.items() if name == topic),
                         extra=sum(n for (name, _), n in extra.items() if name == topic)))
    summary = dict(input=str(args.bag), duration_s=args.duration, mode=args.mode,
                   storage=args.storage, domain=args.domain, started_wall_s=started,
                   player_wall_s=player_elapsed,
                   publisher_drain_s=2,
                   recorder_exit=recorder_results[0]['exit'], rows=rows,
                   recorder_command=recorder_results[0]['command'],
                   separate_imu=args.separate_imu, recorders=recorder_results,
                   qos_file=str(args.qos_file) if args.qos_file else None,
                   missing_payload_examples=missing_examples(args.bag, missing))
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
