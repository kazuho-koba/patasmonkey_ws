#!/usr/bin/env python3
"""単一MCAPと実GUIのeventログから、各購読条件の全topic記録率を比較する。

bagの記録時刻をPCのwall_timeと照合するため、両機の時計同期を前提とする。
各区間の境界付近を除外する。GUIの描画枚数はbag保存件数の代用にしない。
独立プローブは追加しないため、publish全件に対する欠測率は断定できない。
"""
import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

from mcap.reader import make_reader
from audit_bag_delivery import header_stamp, topic_summary


def stages(events, settle=3.0, tail=2.0):
    """OFF→ONは同じGUI内のeventで分割し、終了ごとに定常評価区間を作る。"""
    active = None
    result = []
    for event in events:
        kind = event['event']
        if kind in ('begin', 'camera_on_begin'):
            name = ('camera_off' if event['mode'] == 'offon' and kind == 'begin'
                    else 'camera_on' if kind == 'camera_on_begin' else event['mode'])
            active = (name, event['wall_time'])
        elif kind in ('end', 'camera_off_end') and active:
            name, begin = active
            low, high = begin + settle, event['wall_time'] - tail
            if high <= low:
                raise ValueError('境界除外後の評価区間が空です')
            result.append((name, low, high))
            active = None
    if active:
        raise ValueError('GUI eventの終了がありません')
    if not result:
        raise ValueError('評価できるGUI区間がありません')
    return result


def requirements_failures(rows, intervals, requirements):
    """未記録topicも0 Hzとして判定し、VIO未成立を見逃さない。"""
    failures = []
    for topic, minimum in requirements:
        for name, _, _ in intervals:
            value = next((row['window_hz'] for row in rows if row['stage'] == name and row['topic'] == topic), 0)
            if value < minimum:
                failures.append(dict(stage=name, topic=topic, recorded_hz=value, minimum_hz=minimum))
    return failures


def coverage_failures(intervals, origin, end):
    """記録されていない時間を分母に含む区間を、記録率低下と誤認させない。"""
    return [dict(stage=name, requested_start=low, requested_end=high,
                 bag_start=origin, bag_end=end) for name, low, high in intervals
            if low < origin or high > end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--events', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-topic', action='append', default=[], metavar='TOPIC:MIN_HZ',
                        help='全評価区間で必要な出力の最低記録率。未達でも解析を保存し終了code 1')
    args = parser.parse_args()
    requirements = []
    for specification in args.require_topic:
        try:
            topic, minimum = specification.rsplit(':', 1)
            minimum = float(minimum)
            if not topic or not math.isfinite(minimum) or minimum <= 0:
                raise ValueError()
        except ValueError:
            parser.error('TOPIC:MIN_HZのMIN_HZは有限の正数です')
        requirements.append((topic, minimum))
    intervals = stages([json.loads(line) for line in args.events.read_text().splitlines()])
    args.output.mkdir(exist_ok=False, parents=True)
    rows = defaultdict(list)
    for path in sorted(args.bag.glob('*.mcap')):
        with path.open('rb') as stream:
            for schema, channel, message in make_reader(stream).iter_messages():
                rows[channel.topic].append((header_stamp(message.data, schema.name if schema else ''), message.log_time))
    if not rows:
        raise ValueError('MCAP messageがありません')
    origin = min(log for values in rows.values() for _, log in values)*1e-9
    end = max(log for values in rows.values() for _, log in values)*1e-9
    coverage = coverage_failures(intervals, origin, end)
    invalid = {row['stage'] for row in coverage}
    output = []
    for name, low, high in intervals:
        for topic, values in sorted(rows.items()):
            selected = [(stamp, log) for stamp, log in values if low * 1e9 <= log < high * 1e9]
            metrics = topic_summary(selected)
            # 端点間Hzだけでは途中停止を見逃すため、固定評価区間の件数/秒も保存する。
            output.append(dict(stage=name, topic=topic, coverage_valid=name not in invalid, start_wall_s=low, end_wall_s=high,
                duration_s=high-low, window_hz=len(selected)/(high-low), **metrics))
    fields = list(dict.fromkeys(key for row in output for key in row))
    with (args.output/'stages.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(output)
    (args.output/'summary.json').write_text(json.dumps(output, indent=2)+'\n')
    for row in output:
        if row['topic'] in ('/wit/imu', '/oak/stereo/left/image_raw', '/oak/depth/image_raw', '/odometry/local', '/odometry/global', '/oak/imu/data', '/ov_msckf/odomimu', '/ov_msckf/points_msckf'):
            print('{} {} {:.3f} Hz max_record_gap={} s'.format(row['stage'], row['topic'], row['window_hz'], row['max_record_gap_s']))
    failures = requirements_failures(output, intervals, requirements)
    # センサが正常でもVIO出力が停止した試行を、全体性能の合格と誤認しない。
    (args.output/'requirements.json').write_text(json.dumps(failures, indent=2)+'\n')
    (args.output/'coverage.json').write_text(json.dumps(coverage, indent=2)+'\n')
    if coverage:
        print('INVALID_COVERAGE: 記録範囲外を含む区間があります。固定区間Hzを性能低下と判断しないでください。')
    if failures or coverage:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
