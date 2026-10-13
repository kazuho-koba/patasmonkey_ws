#!/usr/bin/env python3
"""全topicの記録停滞・header空白とOAK sequence診断を読み取り専用で集計する。

単独bagから真のpublish件数は復元できない。周波数や空白は欠測の疑いとして扱い、
診断のsequence gapもカメラ内drop、driver間引き、診断自体の未記録を区別して解釈する。
"""
import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
import struct

from mcap.reader import make_reader
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.serialization import deserialize_message
from pm_evaluation.cli.analyze_bag_frequencies import load_bag_metadata, find_mcap_files
from compare_live_imu_bags import metrics, stamp_seconds, TOPICS


# CDR1で先頭がHeaderの型だけを許可する。TFMessage等の配列先頭をHeaderと誤認しない。
# 大きい画像・点群をROS messageへ展開せず、stamp[ns]だけを読むことで解析負荷を抑える。
HEADER_TYPES = {'sensor_msgs/msg/'+name for name in (
    'Image', 'CameraInfo', 'Imu', 'MagneticField', 'NavSatFix', 'PointCloud2',
    'CompressedImage', 'JointState')}
HEADER_TYPES.update({'nav_msgs/msg/Odometry', 'nav_msgs/msg/Path',
                     'geometry_msgs/msg/PoseStamped', 'geometry_msgs/msg/TwistStamped',
                     'diagnostic_msgs/msg/DiagnosticArray'})


def header_stamp(data, message_type):
    """許可した型・CDR1・妥当なnanosecに限り、整数nsのstampを取り出す。"""
    if message_type not in HEADER_TYPES or len(data) < 12:
        return None
    encoding = bytes(data[:2])
    if encoding not in (b'\x00\x00', b'\x00\x01'):
        return None
    sec, nano = struct.unpack_from('<iI' if encoding == b'\x00\x01' else '>iI', data, 4)
    return sec*1_000_000_000+nano if nano < 1_000_000_000 else None


def topic_summary(rows):
    """記録時計とheader時計を別々に評価し、受信順のheader逆行も残す。"""
    received = sorted(rows, key=lambda row: row[1])
    logs = [row[1] for row in received]
    gaps = [(b-a)*1e-9 for a, b in zip(logs, logs[1:])]
    span = (logs[-1]-logs[0])*1e-9 if logs else 0
    headers = [(stamp, log, '') for stamp, log in received if stamp is not None]
    result = {'count': len(rows), 'record_hz': (len(rows)-1)/span if span > 0 else None,
              'max_record_gap_s': max(gaps) if gaps else None,
              'header_count': len(headers)}
    if headers:
        header_metrics = metrics(sorted(headers))
        header_metrics.pop('count')  # 不正headerがあっても全message件数を減らさない。
        result.update(header_metrics)
    return result


def largest_record_gaps(rows, origin, limit=5):
    """最大空白の位置をbag先頭からの秒で残し、起動時だけの停滞を識別する。"""
    logs = sorted(log for _, log in rows)
    gaps = sorted(zip(logs, logs[1:]), key=lambda pair: pair[1]-pair[0], reverse=True)
    return [dict(start_s=(a-origin)*1e-9, end_s=(b-origin)*1e-9, gap_s=(b-a)*1e-9)
            for a, b in gaps[:limit]]


def gap_evidence(rows, origin, limit=5):
    """記録空白の両端のheaderを照合し、保存遅延と測定空白を混同しない。

    recordはrecorderの受信時計であり、SSD書込み完了時刻ではない。
    header差が小さいのにrecord差が大きい場合も、両端間の全発行数は保証できない。
    同一headerや逆行もそのまま残す。遅延差は絶対時計のオフセットに依存しない。
    """
    ordered = sorted(rows, key=lambda row: row[1])
    pairs = sorted(zip(ordered, ordered[1:]),
                   key=lambda pair: pair[1][1]-pair[0][1], reverse=True)
    result = []
    for (header0, record0), (header1, record1) in pairs[:limit]:
        record_delta = (record1-record0)*1e-9
        header_delta = ((header1-header0)*1e-9
                        if header0 is not None and header1 is not None else None)
        result.append(dict(start_s=(record0-origin)*1e-9,
                           end_s=(record1-origin)*1e-9, record_gap_s=record_delta,
                           header_gap_s=header_delta,
                           receipt_age_change_s=(record_delta-header_delta
                                                 if header_delta is not None else None)))
    return result


def header_retention(source, reference):
    """共通header区間内のmultiset一致率。referenceの完全性は仮定しない。

    重複stampをsetで消さず件数を照合する。連続欠落はreferenceに存在する順序で
    数え、欠落burstの時間は最初/最後の欠落header差とする（1件なら0秒）。
    両系列が同時に失ったデータは検出できないため、全publish保存率は不明とする。
    """
    a = [stamp for stamp, _ in source if stamp is not None]
    b = [stamp for stamp, _ in reference if stamp is not None]
    if not a or not b:
        return {'status': 'missing_topic', 'all_publish_retention_percent': None}
    low, high = max(min(a), min(b)), min(max(a), max(b))
    if high < low:
        return {'status': 'no_common_interval', 'all_publish_retention_percent': None}
    source_counts = Counter(stamp for stamp in a if low <= stamp <= high)
    reference_counts = Counter(stamp for stamp in b if low <= stamp <= high)
    matched = sum((source_counts & reference_counts).values())
    remaining = source_counts.copy()
    longest = length = 0
    longest_span = 0.0
    first_missing = None
    for stamp in sorted(reference_counts.elements()):
        if remaining[stamp] > 0:
            remaining[stamp] -= 1
            length, first_missing = 0, None
        else:
            length += 1
            first_missing = stamp if first_missing is None else first_missing
            longest = max(longest, length)
            longest_span = max(longest_span, (stamp-first_missing)*1e-9)
    total = sum(reference_counts.values())
    return dict(status='ok', source_count=sum(source_counts.values()),
                reference_count=total, matched_count=matched,
                reference_only=total-matched,
                source_only=sum(source_counts.values())-matched,
                observed_retention_percent=(100.0*matched/total if total else None),
                max_consecutive_reference_missing=longest,
                max_missing_header_span_s=longest_span,
                # 小さな欠落の再解析用に最初の20件だけを保存する。全画像payloadは保持しない。
                first_reference_missing_headers_ns=sorted((reference_counts-source_counts).elements())[:20],
                common_header_start_ns=low, common_header_end_ns=high,
                all_publish_retention_percent=None)


def stereo_header_pairs(rows):
    """左右画像の同一stamp対応を双方向で評価。VOが受信した件数とは別である。"""
    left = rows.get('/oak/stereo/left/image_raw', [])
    right = rows.get('/oak/stereo/right/image_raw', [])
    return {'left_against_right': header_retention(left, right),
            'right_against_left': header_retention(right, left)}


def camera_info_header_pairs(rows):
    """現driverが画像ごとに同一stampで発行するCameraInfoの記録対応を確認する。

    colorとdepthは撮像時刻が異なり得るため、相互の完全一致は要求しない。
    固定校正を1回だけ出す別driverのbagではmissingを校正不使用と即断しない。
    """
    result = {}
    for camera in ('color', 'depth'):
        image = rows.get('/oak/'+camera+'/image_raw', [])
        info = rows.get('/oak/'+camera+'/camera_info', [])
        result[camera] = dict(image_against_info=header_retention(image, info),
                              info_against_image=header_retention(info, image))
    return result


def compare_diagnostic_headers(rows):
    """OAKのsourceと診断を共通header区間で照合する。診断自体の欠落も別に数える。"""
    pairs = {'left': '/oak/stereo/left/image_raw', 'right': '/oak/stereo/right/image_raw',
             'color': '/oak/color/image_raw', 'depth': '/oak/depth/image_raw',
             'confidence': '/oak/stereo/confidence/image_raw',
             'disparity': '/oak/stereo/disparity/image_raw', 'imu': '/oak/imu/data'}
    results = {}
    for name, source in pairs.items():
        diagnostic = '/oak/diagnostics/'+('imu_packet' if name == 'imu' else name+'_frame')
        left = [stamp for stamp, _ in rows.get(source, []) if stamp is not None]
        right = [stamp for stamp, _ in rows.get(diagnostic, []) if stamp is not None]
        if not left or not right:
            results[name] = {'status': 'missing_topic'}
            continue
        low, high = max(min(left), min(right)), min(max(left), max(right))
        if high < low:
            results[name] = {'status': 'no_common_interval'}
            continue
        a = Counter(stamp for stamp in left if low <= stamp <= high)
        b = Counter(stamp for stamp in right if low <= stamp <= high)
        results[name] = {**header_retention(rows[source], rows[diagnostic]),
                         'reference_kind': 'recorded_driver_diagnostic_not_all_publishes',
                         'status': 'ok', 'source_count': sum(a.values()),
                         'diagnostic_count': sum(b.values()),
                         'source_only': sum((a-b).values()), 'diagnostic_only': sum((b-a).values())}
    return results


def compare_probe_headers(rows, path):
    """専用bagなしでも、独立IMUプローブとの共通header区間を双方向で照合する。"""
    probe = {topic: Counter() for topic in TOPICS}
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            if row['topic'] in probe:
                probe[row['topic']][float(row['header_s'])] += 1
    result = {}
    for topic in TOPICS:
        bag = Counter(stamp_seconds(stamp) for stamp, _ in rows.get(topic, []) if stamp is not None)
        observed = probe[topic]
        if not bag or not observed:
            result[topic] = {'status': 'missing_topic'}
            continue
        low, high = max(min(bag), min(observed)), min(max(bag), max(observed))
        if high < low:
            result[topic] = {'status': 'no_common_interval'}
            continue
        a = Counter({stamp: count for stamp, count in bag.items() if low <= stamp <= high})
        b = Counter({stamp: count for stamp, count in observed.items() if low <= stamp <= high})
        result[topic] = {'status': 'ok', 'bag_count': sum(a.values()), 'probe_count': sum(b.values()),
                         'probe_only': sum((b-a).values()), 'bag_only': sum((a-b).values())}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--additional-bag', type=Path, action='append', default=[],
                        help='排他的topic分離の追加MCAP。重複topicは拒否する')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profiles', type=Path, help='未記録topic確認用bag_profiles.yaml')
    parser.add_argument('--probe-csv', type=Path, help='独立IMUプローブのheader照合（任意）')
    parser.add_argument('--profile', default='mission')
    parser.add_argument('--window-s', type=float, default=5.0)
    parser.add_argument('--settle-s', type=float, default=10.0,
                        help='bag先頭から除外する起動区間[秒]。全期間の結果も保持する')
    parser.add_argument('--tail-s', type=float, default=2.0,
                        help='bag末尾から除外する停止区間[秒]')
    parser.add_argument('--end-s', type=float, default=None,
                        help='定常区間の終了をbag先頭からの秒で制限する（複数bagの同区間比較）')
    args = parser.parse_args()
    if not math.isfinite(args.window_s) or args.window_s <= 0:
        parser.error('--window-sは有限の正の秒数です')
    if any(not math.isfinite(value) or value < 0 for value in (args.settle_s, args.tail_s)):
        parser.error('--settle-sと--tail-sは有限の非負秒数です')
    if args.end_s is not None and (not math.isfinite(args.end_s) or args.end_s < 0):
        parser.error('--end-sは有限の非負秒数です')
    if args.output.exists():
        parser.error('既存解析を上書きしません')
    rows = defaultdict(list)
    sequence = defaultdict(lambda: defaultdict(list))
    types = {}
    file_ranges = []
    directories = [args.bag, *args.additional_bag]
    if len({path.resolve() for path in directories}) != len(directories):
        parser.error('同じbagを複数回指定できません')
    files = []
    for directory in directories:
        metadata = load_bag_metadata(directory/'metadata.yaml')
        if metadata.storage_identifier != 'mcap':
            parser.error('MCAP bagだけを扱います')
        selected = find_mcap_files(metadata)
        if not selected or any(not (directory/name).is_file() for name in metadata.relative_file_paths):
            parser.error('metadataに対応するbagファイルが不足しています')
        files.extend((directory.resolve(), path) for path in selected)
    owners = {}
    # 各bag内の分割ファイルを読み、排他的分離だけを合算する。IMU重複保存を
    # 指定すると件数が二倍になるため、別bag間で同じtopicが現れたら停止する。
    for directory, path in files:
        first_log = last_log = None
        with path.open('rb') as stream:
            for schema, channel, record in make_reader(stream).iter_messages():
                if channel.topic in owners and owners[channel.topic] != directory:
                    parser.error('複数bagに同じtopicがあります: '+channel.topic)
                owners[channel.topic] = directory
                first_log = record.log_time if first_log is None else min(first_log, record.log_time)
                last_log = record.log_time if last_log is None else max(last_log, record.log_time)
                kind = schema.name if schema else ''
                types[channel.topic] = kind
                rows[channel.topic].append((header_stamp(record.data, kind), record.log_time))
                if kind == 'diagnostic_msgs/msg/DiagnosticArray' and channel.topic.startswith('/oak/diagnostics/'):
                    msg = deserialize_message(record.data, DiagnosticArray)
                    for status in msg.status:
                        for item in status.values:
                            if 'sequence' in item.key:
                                try:
                                    value = int(item.value)
                                except ValueError:
                                    continue
                                sequence[channel.topic][status.name+'/'+item.key].append((record.log_time, value))
        file_ranges.append({'file': path.name, 'bag': str(directory), 'bytes': path.stat().st_size,
                            'first_log_ns': first_log, 'last_log_ns': last_log})
    origin = min((log for values in rows.values() for _, log in values), default=0)
    windows = []
    width = round(args.window_s*1e9)
    for topic, values in sorted(rows.items()):
        bins = Counter((log-origin)//width for _, log in values)
        last = max(bins, default=-1)
        for index in range(last+1):
            # 先頭・末尾windowはtopic起動待ちや停止が混ざる。固定幅のcountを保存し、
            # 欠測数や厳密なpublish Hzへ自動変換しない。
            windows.append({'topic': topic, 'start_s': index*args.window_s,
                            'end_s': (index+1)*args.window_s, 'count': bins[index]})
    seq_summary = {}
    for topic, fields in sequence.items():
        seq_summary[topic] = {}
        for key, values in fields.items():
            ordered = [value for _, value in sorted(values)]
            seq_summary[topic][key] = {'first': ordered[0], 'last': ordered[-1],
                                      'delta': ordered[-1]-ordered[0],
                                      'max': max(ordered), 'count': len(ordered)}
    summaries = {topic: dict(message_type=types[topic], **topic_summary(values))
                 for topic, values in sorted(rows.items())}
    # 全topicに同じ記録時計の範囲を適用する。topicごとの開始を原点にすると、
    # 遅い購読成立を隠してしまうため、bag全体の先頭/末尾を使う。
    end = max((log for values in rows.values() for _, log in values), default=origin)
    low, high = origin+round(args.settle_s*1e9), end-round(args.tail_s*1e9)
    if args.end_s is not None:
        # 第二bagの停止がmainより遅くても、同じbag先頭相対秒で比較する。
        high = min(high, origin+round(args.end_s*1e9))
    steady_rows = {topic: [(stamp, log) for stamp, log in values if low <= log <= high]
                   for topic, values in sorted(rows.items())}
    steady = {topic: topic_summary(values) for topic, values in steady_rows.items()}
    missing = []
    if args.profiles:
        import yaml
        expected = yaml.safe_load(args.profiles.read_text())[args.profile]
        missing = sorted(set(expected)-set(rows))
    args.output.mkdir(parents=True)
    (args.output/'summary.json').write_text(json.dumps({
        'bag': str(args.bag), 'topics': summaries, 'unrecorded_profile_topics': missing,
        'input_bags': [str(path) for path in directories],
        'steady_interval': {'start_s': args.settle_s, 'end_s': (high-origin)*1e-9,
                            'valid': high >= low},
        'steady_topics': steady,
        'steady_independent_imu_probe': (compare_probe_headers(steady_rows, args.probe_csv)
                                         if args.probe_csv else None),
        'steady_source_diagnostic_headers': compare_diagnostic_headers(steady_rows),
        'steady_stereo_header_pairs': stereo_header_pairs(steady_rows),
        'steady_camera_info_header_pairs': camera_info_header_pairs(steady_rows),
        'record_gap_evidence': {topic: gap_evidence(values, origin)
                               for topic, values in sorted(rows.items())},
        'largest_record_gaps': {topic: largest_record_gaps(values, origin)
                                for topic, values in sorted(rows.items())},
        'file_record_ranges': file_ranges,
        'source_diagnostic_headers': compare_diagnostic_headers(rows),
        'independent_imu_probe': compare_probe_headers(rows, args.probe_csv) if args.probe_csv else None,
        'sequence_diagnostics': seq_summary}, indent=2)+'\n')
    with (args.output/'windows.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=['topic', 'start_s', 'end_s', 'count'])
        writer.writeheader()
        writer.writerows(windows)
    print(json.dumps(summaries, indent=2), flush=True)


if __name__ == '__main__':
    main()
