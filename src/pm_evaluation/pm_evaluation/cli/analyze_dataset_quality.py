#!/usr/bin/env python3
"""MCAP原本を変更せず、時間窓ごとの保存品質と利用可能区間を出力する。

画像本文は展開せず、記録時刻と標準メッセージ先頭のHeaderだけを読む。
ROSノード・センサ・モータは起動しない。全時刻は元の共通時刻系を保持する。
"""
import argparse
import bisect
import csv
import hashlib
import json
import math
import struct
from datetime import datetime, timezone
from pathlib import Path

import yaml
from mcap.reader import make_reader


VERSION = '1.0'
HEADER_TYPES = {'sensor_msgs/msg/Imu', 'sensor_msgs/msg/Image',
                'sensor_msgs/msg/MagneticField', 'sensor_msgs/msg/NavSatFix',
                'nav_msgs/msg/Odometry', 'geometry_msgs/msg/TwistWithCovarianceStamped',
                'geometry_msgs/msg/TwistStamped', 'sensor_msgs/msg/CameraInfo'}


def load_policy(path):
    """誤設定による自動合格を防ぎ、未確定Hzは明示的に未判定とする。"""
    policy = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(policy, dict) or policy.get('version') != 1:
        raise ValueError('policy.versionは1が必要です')
    for key in ('window_sec', 'step_sec'):
        value = policy.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(key + 'は正の有限値が必要です')
    if policy['step_sec'] > policy['window_sec']:
        raise ValueError('step_secはwindow_sec以下にしてください')
    count = policy.get('consecutive_windows', 3)
    if not isinstance(count, int) or isinstance(count, bool) or count < 1:
        raise ValueError('consecutive_windowsは正の整数が必要です')
    policy['consecutive_windows'] = count
    if not isinstance(policy.get('topics'), dict) or not policy['topics']:
        raise ValueError('topicsが必要です')
    if not all(isinstance(c, dict) for c in policy['topics'].values()):
        raise ValueError('各topicの設定は辞書にしてください')
    if not any(c.get('required', False) for c in policy['topics'].values()):
        raise ValueError('少なくとも1つのrequiredトピックが必要です')
    for topic, config in policy['topics'].items():
        if not topic.startswith('/') or config.get('clock', 'record') not in ('record', 'header'):
            raise ValueError('topic名またはclockが不正です: ' + topic)
        for key in ('required', 'header', 'rate_check'):
            if key in config and not isinstance(config[key], bool):
                raise ValueError(key + 'はYAMLのtrue/falseで指定してください')
        hz = config.get('expected_hz')
        if hz is not None and (not isinstance(hz, (int, float)) or not math.isfinite(hz) or hz <= 0):
            raise ValueError('expected_hzは正数またはnullです: ' + topic)
        tolerance = config.get('tolerance_ratio', .01)
        if not 0 <= tolerance < 1:
            raise ValueError('tolerance_ratioは0以上1未満です')
        for key in ('max_gap_sec', 'max_delay_sec', 'max_future_sec'):
            if key in config and (not math.isfinite(config[key]) or config[key] <= 0):
                raise ValueError(key + 'は正の有限値です')
        if config.get('clock') == 'header' and not config.get('header', False):
            raise ValueError('header clockにはheader: trueが必要です')
    stereo = policy.get('stereo')
    if stereo:
        if not isinstance(stereo.get('required', False), bool):
            raise ValueError('stereo.requiredはtrue/falseで指定してください')
        if any(t not in policy['topics'] or not policy['topics'][t].get('header')
               for t in (stereo['left'], stereo['right'])):
            raise ValueError('左右topicをheader: trueでtopicsに定義してください')
        if not 0 <= stereo['min_match_ratio'] <= 1 or stereo['max_delta_sec'] < 0:
            raise ValueError('stereoの許容値が不正です')
    return policy


def read_bag(bag, policy):
    """全分割を厳密確認して読む。画像データを保持せず時刻列だけ保存する。"""
    metadata_bytes = (bag / 'metadata.yaml').read_bytes()
    meta = yaml.safe_load(metadata_bytes)['rosbag2_bagfile_information']
    if meta['storage_identifier'] != 'mcap':
        raise ValueError('MCAP形式のみ対応しています')
    names = meta['relative_file_paths']
    if not names or len(set(names)) != len(names):
        raise ValueError('分割ファイル一覧が空または重複しています')
    paths = [(bag / name).resolve() for name in names]
    if any(bag.resolve() not in p.parents or not p.is_file() for p in paths):
        raise ValueError('分割ファイルが欠損、またはbag外を参照しています')
    origin = int(meta['starting_time']['nanoseconds_since_epoch'])
    duration = int(meta['duration']['nanoseconds']) * 1e-9
    if duration <= 0:
        raise ValueError('bag記録時間が正ではありません')
    rows = {topic: [] for topic in policy['topics']}
    errors = {topic: 0 for topic in rows}
    for path in paths:
        with path.open('rb') as stream:
            for schema, channel, msg in make_reader(stream).iter_messages(topics=list(rows)):
                config = policy['topics'][channel.topic]
                stamp = None
                if config.get('header', False):
                    try:
                        name = schema.name.replace('/msg/', '/') if schema else ''
                        allowed = {t.replace('/msg/', '/') for t in HEADER_TYPES}
                        if name not in allowed or channel.message_encoding != 'cdr':
                            raise ValueError('未対応のHeader型')
                        if msg.data[:2] not in (b'\x00\x00', b'\x00\x01'):
                            raise ValueError('未対応のCDR表現')
                        sec, ns = struct.unpack_from(('<' if msg.data[1] else '>') + 'iI', msg.data, 4)
                        if not 0 <= ns < 10**9 or sec * 10**9 + ns <= 0:
                            raise ValueError('Header.stampが無効です')
                        stamp = sec * 10**9 + ns
                    except (ValueError, struct.error):
                        errors[channel.topic] += 1
                rows[channel.topic].append((msg.log_time, stamp))
    expected = {e['topic_metadata']['name']: int(e['message_count'])
                for e in meta['topics_with_message_count']}
    for topic in rows:
        if len(rows[topic]) != expected.get(topic, 0):
            raise ValueError('metadata件数不一致: ' + topic)
        rows[topic].sort(key=lambda row: row[0])
    identity = {'bag': str(bag.resolve()), 'metadata_sha256': hashlib.sha256(metadata_bytes).hexdigest(),
                'files': [{'name': p.name, 'size_bytes': p.stat().st_size} for p in paths],
                'starting_time_ns': origin, 'duration_sec': duration}
    return origin, duration, rows, errors, identity


def gap_in_window(times, start, end):
    """窓境界をまたぐ欠測も検査する。窓外の遠い欠測を過大評価しない。"""
    lo, hi = bisect.bisect_left(times, start), bisect.bisect_left(times, end)
    inside = times[lo:hi]
    # 境界をまたぐ長い間隔を半分ずつに切って正常と見なさない。
    edges = [times[lo-1] if lo else start, *inside,
             times[hi] if hi < len(times) else end]
    return max((b - a for a, b in zip(edges, edges[1:])), default=end-start)


def intervals(windows, required_count, origin):
    """連続合格runを抽出。重複窓の和集合内だけを暫定利用可能とする。"""
    result, run = [], []
    for window in windows + [None]:
        if window is not None and window['passed']:
            run.append(window)
            continue
        if len(run) >= required_count:
            start, end, confirmed = run[0]['start_sec'], run[-1]['end_sec'], run[required_count-1]['end_sec']
            result.append({'start_sec': start, 'end_sec': end, 'confirmed_at_sec': confirmed,
                           'start_timestamp_ns': origin + round(start * 1e9),
                           'end_timestamp_ns': origin + round(end * 1e9)})
        run = []
    return result


def analyze(origin, duration, rows, errors, policy):
    """固定長窓の件数Hzで判定し、Headerの間隔Hzも診断値として併記する。"""
    prepared = {}
    for topic, values in rows.items():
        record = [(t-origin)*1e-9 for t, _ in values]
        headers_in_record_order = [(h-origin)*1e-9 for _, h in values if h is not None]
        header = sorted(headers_in_record_order)
        # bag記録順での逆行を残す。ソートしてから検査すると異常を隠してしまう。
        reversed_at = [record[i] for i in range(1, len(values))
                       if values[i][1] is not None and values[i-1][1] is not None
                       and values[i][1] <= values[i-1][1]]
        prepared[topic] = (record, header, reversed_at)
    table, global_windows, rate_windows = [], [], []
    topic_windows = {topic: [] for topic in rows}
    for index in range(max(0, int(math.floor((duration-policy['window_sec']) / policy['step_sec'] + 1e-8)) + 1)):
        start = index * policy['step_sec']
        end = start + policy['window_sec']
        global_reasons = []
        rate_reasons = []
        for topic, config in policy['topics'].items():
            record, header, reversed_at = prepared[topic]
            selected = header if config.get('clock', 'record') == 'header' else record
            lo, hi = bisect.bisect_left(selected, start), bisect.bisect_left(selected, end)
            part = selected[lo:hi]
            hz = len(part) / policy['window_sec']
            hlo, hhi = bisect.bisect_left(header, start), bisect.bisect_left(header, end)
            hp = header[hlo:hhi]
            header_hz = (len(hp)-1)/(hp[-1]-hp[0]) if len(hp)>1 and hp[-1]>hp[0] else None
            rlo, rhi = bisect.bisect_left(record, start), bisect.bisect_left(record, end)
            delays = [(r-h)*1e-9 for r, h in rows[topic][rlo:rhi] if h is not None]
            gap = gap_in_window(selected, start, end)
            reasons = []
            if not part:
                reasons.append('no_messages')
            if config.get('rate_check', True):
                expected = config.get('expected_hz')
                if expected is None:
                    reasons.append('expected_hz_unconfigured')
                elif abs(hz / expected - 1) > config.get('tolerance_ratio', .01) + 1e-12:
                    reasons.append('rate_out_of_range')
            if gap > config.get('max_gap_sec', float('inf')) + 1e-12:
                reasons.append('gap_too_large')
            if any(start <= t < end for t in reversed_at):
                reasons.append('header_not_increasing')
            if config.get('header') and len(delays) != rhi-rlo:
                reasons.append('invalid_header')
            if delays and min(delays) < -config.get('max_future_sec', .001):
                reasons.append('header_in_future')
            if delays and max(delays) > config.get('max_delay_sec', float('inf')):
                reasons.append('record_delay_too_large')
            item = {'topic': topic, 'start_sec': start, 'end_sec': end,
                    'record_count': rhi-rlo, 'header_count': hhi-hlo,
                    'record_hz': (rhi-rlo)/policy['window_sec'],
                    'header_count_hz': (hhi-hlo)/policy['window_sec'],
                    'header_interval_hz': header_hz, 'selected_hz': hz,
                    'max_gap_sec': gap, 'max_delay_sec': max(delays) if delays else None,
                    'min_delay_sec': min(delays) if delays else None,
                    'required': config.get('required', False), 'passed': not reasons,
                    'reasons': ';'.join(reasons)}
            table.append(item)
            topic_windows[topic].append(item)
            if item['required'] and reasons:
                global_reasons.append(topic + ':' + ';'.join(reasons))
                # 周波数だけの判断と追加品質チェックを分離して保存する。
                rate_reasons.extend(topic + ':' + reason for reason in reasons
                                    if reason in ('no_messages', 'expected_hz_unconfigured', 'rate_out_of_range'))
        stereo = policy.get('stereo')
        if stereo:
            left, right = prepared[stereo['left']][1], prepared[stereo['right']][1]
            left = left[bisect.bisect_left(left, start):bisect.bisect_left(left, end)]
            right = right[bisect.bisect_left(right, start):bisect.bisect_left(right, end)]
            i = j = matched = 0
            while i < len(left) and j < len(right):
                delta = left[i] - right[j]
                if abs(delta) <= stereo['max_delta_sec']:
                    matched += 1
                    i += 1
                    j += 1
                elif delta < 0:
                    i += 1
                else:
                    j += 1
            ratio = matched/max(len(left), len(right)) if left or right else 0
            reason = '' if ratio >= stereo['min_match_ratio'] and matched else 'stereo_unmatched'
            table.append({'topic': '@stereo', 'start_sec': start, 'end_sec': end,
                          'required': stereo.get('required', False), 'passed': not reason,
                          'reasons': reason, 'stereo_match_ratio': ratio})
            if reason and stereo.get('required'):
                global_reasons.append(reason)
        global_windows.append({'start_sec': start, 'end_sec': end,
                               'passed': not global_reasons, 'reasons': global_reasons})
        rate_windows.append({'start_sec': start, 'end_sec': end,
                             'passed': not rate_reasons, 'reasons': rate_reasons})
    valid = intervals(global_windows, policy['consecutive_windows'], origin)
    rate_valid = intervals(rate_windows, policy['consecutive_windows'], origin)
    return table, {'status': 'provisional_pass' if valid else 'no_qualified_interval',
                   'valid_intervals': valid, 'stable_start_sec': valid[0]['start_sec'] if valid else None,
                   'confirmed_at_sec': valid[0]['confirmed_at_sec'] if valid else None,
                   'rate_only_valid_intervals': rate_valid,
                   'rate_only_stable_start_sec': rate_valid[0]['start_sec'] if rate_valid else None,
                   'rate_only_confirmed_at_sec': rate_valid[0]['confirmed_at_sec'] if rate_valid else None,
                   'rate_only_windows': rate_windows,
                   'global_windows': global_windows,
                   'unassessed_tail_start_sec': global_windows[-1]['end_sec'] if global_windows else 0,
                   'topics': {topic: {'required': policy['topics'][topic].get('required', False),
                                      'count': len(values), 'invalid_header_count': errors[topic],
                                      'first_record_sec': prepared[topic][0][0] if values else None,
                                      'last_record_sec': prepared[topic][0][-1] if values else None,
                                      'valid_intervals': intervals(topic_windows[topic], policy['consecutive_windows'], origin)}
                              for topic, values in rows.items()}}


def save_outputs(output, policy, table, summary):
    """派生ファイルだけ保存する。PNGは外部ウィンドウを開かず生成する。"""
    output.mkdir(parents=True, exist_ok=False)
    (output/'policy.yaml').write_text(yaml.safe_dump(policy, allow_unicode=True, sort_keys=False), encoding='utf-8')
    (output/'summary.yaml').write_text(yaml.safe_dump(summary, allow_unicode=True, sort_keys=False), encoding='utf-8')
    fields = ['topic', 'start_sec', 'end_sec', 'record_count', 'header_count', 'record_hz',
              'header_count_hz', 'header_interval_hz', 'selected_hz', 'max_gap_sec',
              'max_delay_sec', 'min_delay_sec', 'stereo_match_ratio', 'required', 'passed', 'reasons']
    with (output/'topic_windows.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(table)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for topic, config in policy['topics'].items():
        values = [row for row in table if row['topic'] == topic]
        expected = config.get('expected_hz')
        if expected and values:
            axes[0].plot([v['end_sec'] for v in values],
                         [v['selected_hz']/expected for v in values], label=topic)
    axes[0].axhspan(.99, 1.01, alpha=.15, color='green', label='reference ±1%')
    axes[0].set_ylabel('recorded / expected rate')
    axes[0].legend(fontsize=7, loc='best')
    windows = summary['global_windows']
    axes[1].step([v['end_sec'] for v in windows], [int(v['passed']) for v in windows], where='post')
    rates = summary['rate_only_windows']
    axes[1].step([v['end_sec'] for v in rates], [int(v['passed']) for v in rates],
                 where='post', alpha=.6, label='rate only')
    axes[1].legend()
    axes[1].set_ylabel('required checks passed')
    axes[1].set_xlabel('seconds from bag start (window end)')
    axes[1].set_yticks([0, 1])
    for axis in axes:
        axis.grid(True, alpha=.3)
    fig.tight_layout()
    fig.savefig(output/'timeline.png', dpi=150)
    plt.close(fig)
    # 最後に完了マーカーを書く。解析失敗時の中途出力を完成と誤認しない。
    (output/'completed.json').write_text(json.dumps({'complete': True, 'analysis_version': VERSION}), encoding='utf-8')


def main(argv=None):
    """1bagずつ明示指定する。原本と既存品質出力の上書きは許可しない。"""
    parser = argparse.ArgumentParser(description='MCAPデータセットの時間別保存品質を解析')
    parser.add_argument('bag', type=Path)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--output', type=Path, help='既定はbag内quality。既存ディレクトリは上書きしない')
    args = parser.parse_args(argv)
    try:
        bag = args.bag.expanduser().resolve()
        output = args.output.expanduser().resolve() if args.output else bag/'quality'
        if output.exists():
            raise ValueError('出力先が既存です。別の--outputを指定してください')
        policy_path = args.policy.expanduser().resolve()
        policy = load_policy(policy_path)
        origin, duration, rows, errors, identity = read_bag(bag, policy)
        table, summary = analyze(origin, duration, rows, errors, policy)
        summary.update({'schema_version': 1, 'analysis_version': VERSION,
                        'analyzed_at_utc': datetime.now(timezone.utc).isoformat(),
                        'source': identity, 'policy_sha256': hashlib.sha256(policy_path.read_bytes()).hexdigest(),
                        'analysis_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                        'limitations': ['保存品質の暫定判定。画像品質・GNSS精度・センサ内部同期は未評価',
                                        'OAK IMUなどexpected_hz未設定のtopicは合格としない',
                                        '末尾の固定長窓を作れない部分は未評価']})
        save_outputs(output, policy, table, summary)
        print(str(output/'summary.yaml'))
        print(summary['status'])
        return 0
    except (ValueError, KeyError, OSError, yaml.YAMLError) as error:
        parser.exit(2, '解析エラー: ' + str(error) + '\n')


if __name__ == '__main__':
    raise SystemExit(main())
