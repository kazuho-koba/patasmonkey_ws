"""bagを一度だけ走査し、保存整合性・時間窓Hz・時刻異常を集計する。

ROS再生は行わない。原本は読み取り専用で、結果は別directoryへ保存する。
MCAPのpayloadはdeserializeせず、既存品質解析と同じ標準Header型だけ先頭を読む。
"""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import sqlite3
import struct
import time

import yaml
from pm_evaluation.cli.analyze_dataset_quality import HEADER_TYPES


def atomic_json(path, data):
    """進捗や結果の途中書込みをUIに見せない。"""
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    temporary.replace(path)


def bag_files(bag, metadata):
    """metadataの順序を保ち、参照がbag外へ出る場合は拒否する。"""
    paths = []
    for name in metadata['relative_file_paths']:
        path = (bag/name).resolve()
        path.relative_to(bag)
        if path in paths or not path.is_file() or path.stat().st_size == 0:
            raise ValueError('分割ファイルの不足・重複・空ファイル: '+str(path))
        paths.append(path)
    if not paths:
        raise ValueError('分割ファイルがありません')
    return paths


def records(paths, storage):
    """MCAP/SQLiteを順番に読み、payloadのコピーや全件保持を避ける。"""
    for path in paths:
        if storage == 'mcap':
            from mcap.reader import make_reader
            with path.open('rb') as stream:
                for schema, channel, message in make_reader(
                        stream, validate_crcs=True).iter_messages(log_time_order=False):
                    yield channel.topic, schema.name if schema else '', message.log_time, message.data
        elif storage == 'sqlite3':
            database = sqlite3.connect(path.as_uri()+'?mode=ro', uri=True)
            try:
                if database.execute('PRAGMA quick_check').fetchone()[0] != 'ok':
                    raise ValueError('SQLite構造検査に失敗しました')
                for topic, kind, stamp, data in database.execute(
                        'SELECT t.name,t.type,m.timestamp,m.data FROM messages m '
                        'LEFT JOIN topics t ON t.id=m.topic_id ORDER BY m.id'):
                    if topic is None:
                        raise ValueError('topic情報のないSQLite messageがあります')
                    yield topic, kind, stamp, data
            finally:
                database.close()
        else:
            raise ValueError('未対応storage: '+storage)


def inspect(bag, output, window_sec=10., progress=None):
    """record時刻の固定窓とHeader異常を集計する。閾値未指定のHzは合否判定しない。"""
    bag, output = Path(bag).resolve(), Path(output).resolve()
    if bag == output or bag in output.parents:
        raise ValueError('結果はbag原本の外へ保存してください')
    if not math.isfinite(window_sec) or window_sec <= 0:
        raise ValueError('時間窓は正の有限値が必要です')
    output.mkdir(parents=True, exist_ok=False)
    info = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    paths = bag_files(bag, info)
    # 走査中に記録が再開・変更された場合に、完了済み原本と誤認しない。
    watched = [bag/'metadata.yaml', *paths]
    before = [(p.stat().st_size, p.stat().st_mtime_ns) for p in watched]
    origin = int(info['starting_time']['nanoseconds_since_epoch'])
    duration = max(0., int(info['duration']['nanoseconds'])/1e9)
    # 長時間bagでもtopicごとの窓数を有界にし、実際の窓幅を結果へ明記する。
    window = max(window_sec, math.ceil(duration/1000.) if duration else window_sec)
    bins = max(1, min(1001, math.ceil(duration/window)))
    expected = {}
    types = {}
    for entry in info['topics_with_message_count']:
        name = entry['topic_metadata']['name']
        if name in expected:
            raise ValueError('metadataに重複topicがあります')
        expected[name] = int(entry['message_count'])
        types[name] = entry['topic_metadata']['type']
    if any(count < 0 for count in expected.values()) or duration < 0:
        raise ValueError('metadataの件数または記録時間が不正です')
    actual, stats = Counter(), {}
    previous_notice = 0.
    header_types = {name.replace('/msg/', '/') for name in HEADER_TYPES}
    for topic, kind, stamp, data in records(paths, info['storage_identifier']):
        row = stats.setdefault(topic, dict(count=0, first_ns=None, last_ns=None,
            max_gap_sec=0., record_backwards=0, header_duplicates=0,
            header_backwards=0, header_invalid=0, header_future=0,
            max_header_gap_sec=0., max_record_header_delay_sec=None,
            header_supported=kind.replace('/msg/', '/') in header_types,
            type=kind, windows=Counter(), previous_record=None, previous_header=None))
        actual[topic] += 1
        row['count'] += 1
        row['first_ns'] = stamp if row['first_ns'] is None else min(row['first_ns'], stamp)
        row['last_ns'] = stamp if row['last_ns'] is None else max(row['last_ns'], stamp)
        previous = row['previous_record']
        if previous is not None:
            delta = (stamp-previous)/1e9
            row['record_backwards'] += int(delta < 0)
            row['max_gap_sec'] = max(row['max_gap_sec'], delta)
        row['previous_record'] = stamp
        index = math.floor((stamp-origin)/1e9/window)
        if stamp == origin+int(info['duration']['nanoseconds']):
            index = min(index, bins-1)
        # 壊れた時刻による無制限のbin生成を防止する。範囲外は異常として残す。
        if 0 <= index <= 1000:
            row['windows'][index] += 1
        else:
            row['record_outside_metadata'] = row.get('record_outside_metadata', 0)+1
        if row['header_supported']:
            try:
                if data[:2] not in (b'\x00\x00', b'\x00\x01'):
                    raise ValueError('未対応CDR')
                sec, ns = struct.unpack_from(('<' if data[1] else '>')+'iI', data, 4)
                header = sec*10**9+ns
                if header <= 0 or ns >= 10**9:
                    raise ValueError('無効Header')
                old = row['previous_header']
                if old is not None:
                    row['header_duplicates'] += int(header == old)
                    row['header_backwards'] += int(header < old)
                    row['max_header_gap_sec'] = max(row['max_header_gap_sec'], (header-old)/1e9)
                row['previous_header'] = header
                delay = (stamp-header)/1e9
                row['header_future'] += int(delay < -.2)
                old_delay = row['max_record_header_delay_sec']
                row['max_record_header_delay_sec'] = delay if old_delay is None else max(old_delay, delay)
            except (ValueError, struct.error):
                row['header_invalid'] += 1
        if progress and time.monotonic()-previous_notice >= .5:
            previous_notice = time.monotonic()
            progress(dict(bag=str(bag), processed=sum(actual.values()), expected=int(info['message_count'])))
    after = [(p.stat().st_size, p.stat().st_mtime_ns) for p in watched]
    if after != before:
        raise ValueError('走査中にbagが変更されました。記録を停止してから再実行してください')
    differences = {name: dict(expected=expected.get(name, 0), actual=actual[name])
                   for name in set(expected)|set(actual) if expected.get(name, 0) != actual[name]}
    type_differences = {name: dict(expected=types.get(name), actual=row['type'])
                        for name, row in stats.items()
                        if types.get(name, '').replace('/msg/', '/') != row['type'].replace('/msg/', '/')}
    for name in expected:
        if name not in stats:
            stats[name] = dict(count=0, type=types[name], windows=Counter(),
                               first_ns=None, last_ns=None, header_supported=False)
    for row in stats.values():
        row.pop('previous_record', None)
        row.pop('previous_header', None)
        span = (row['last_ns']-row['first_ns'])/1e9 if row['count'] else 0
        row['average_hz'] = (row['count']-1)/span if row['count'] > 1 and span > 0 else None
        row['window_hz'] = [
            row['windows'].get(i, 0)/min(window, duration-i*window)
            if duration-i*window > 0 else 0. for i in range(bins)]
        row.pop('windows')
    integrity = (not differences and not type_differences
                 and int(info['message_count']) == sum(actual.values()) == sum(expected.values()))
    issues = sum(sum(row.get(key, 0) for key in (
        'record_backwards', 'header_duplicates', 'header_backwards', 'header_invalid',
        'header_future', 'record_outside_metadata')) for row in stats.values())
    report = dict(version=1, bag=str(bag), storage=info['storage_identifier'],
                  integrity_verified=integrity, crc_checked_when_present=info['storage_identifier']=='mcap',
                  expected_total=int(info['message_count']), actual_total=sum(actual.values()),
                  differences=differences, type_differences=type_differences, timestamp_issue_count=issues,
                  duration_sec=duration, window_sec=window, topics=stats,
                  note='平均Hzと最大空白は診断値。期待Hz未指定のためレート合否は未判定。')
    with (output/'topics.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['topic', 'count', 'average_hz', 'max_gap_sec', 'header_backwards', 'header_invalid'])
        for name, row in sorted(stats.items()):
            writer.writerow([name, row['count'], row['average_hz'], row.get('max_gap_sec'),
                             row.get('header_backwards'), row.get('header_invalid')])
    with (output/'timeline.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['topic', 'start_sec', 'window_hz'])
        for name, row in sorted(stats.items()):
            writer.writerows((name, i*window, hz) for i, hz in enumerate(row['window_hz']))
    atomic_json(output/'report.json', report)
    atomic_json(output/'completed.json', dict(completed=True, integrity_verified=integrity))
    if progress:
        progress(dict(bag=str(bag), processed=sum(actual.values()), expected=int(info['message_count'])))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--window-sec', type=float, default=10.)
    parser.add_argument('--progress')
    args = parser.parse_args()
    try:
        inspect(args.bag, args.output, args.window_sec,
                (lambda data: atomic_json(args.progress, data)) if args.progress else None)
    except Exception as error:
        if args.progress:
            atomic_json(args.progress, dict(error=str(error)))
        raise


if __name__ == '__main__':
    main()
