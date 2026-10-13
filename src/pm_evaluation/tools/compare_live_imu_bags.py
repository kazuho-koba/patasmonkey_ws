#!/usr/bin/env python3
"""同時計測したmission/IMU専用MCAPを共通header区間で照合する。

Foxy環境で入力bagを読み取り専用で扱うCLI。ROSノードやセンサを起動しない。
記録開始・終了時刻の違いを新規欠測と誤認しないよう、topicごとの共通区間を
求めてからtopicとpayload SHA256の多重集合を比較する。片方のbagを真値とは
みなさず、双方にしかないmessageを別々に数える。
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


TOPICS = ('/oak/imu/data', '/wit/imu')


def read_imu(bag):
    """画像を展開せず、header[ns]・記録時刻[ns]・payload署名だけを保持する。"""
    rows = {topic: [] for topic in TOPICS}
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError('metadata.yamlのないbagは比較しません: {}'.format(bag))
    for path in sorted(bag.glob('*.mcap')):
        with path.open('rb') as stream:
            for _, channel, record in make_reader(stream).iter_messages(topics=TOPICS):
                msg = deserialize_message(record.data, Imu)
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                # floatの丸めで元headerを変えず、区間判定は整数nsで行う。
                rows[channel.topic].append((stamp, record.log_time,
                                            hashlib.sha256(record.data).hexdigest()))
    for values in rows.values():
        values.sort()
    return rows


def metrics(rows):
    """header順の測定周期と記録順の停滞を分け、時計の逆順も隠さず返す。"""
    if not rows:
        return dict(count=0, header_active_hz=None, max_header_gap_s=None,
                    median_record_age_s=None, max_record_age_s=None,
                    max_log_gap_s=None, record_order_nonpositive_header_intervals=0)
    stamps = [item[0] for item in rows]
    gaps = [(b - a) * 1e-9 for a, b in zip(stamps, stamps[1:])]
    ages = [(log - stamp) * 1e-9 for stamp, log, _ in rows]
    span = (stamps[-1] - stamps[0]) * 1e-9
    receipt_order = sorted(rows, key=lambda row: row[1])
    log_gaps = [(b[1] - a[1]) * 1e-9 for a, b in zip(receipt_order, receipt_order[1:])]
    return dict(count=len(rows), header_active_hz=(len(rows) - 1) / span if span > 0 else None,
                max_header_gap_s=max(gaps) if gaps else None,
                median_record_age_s=statistics.median(ages), max_record_age_s=max(ages),
                max_log_gap_s=max(log_gaps) if log_gaps else None,
                record_order_nonpositive_header_intervals=sum(
                    b[0] <= a[0] for a, b in zip(receipt_order, receipt_order[1:])))


def compare_topic(mission, sidecar, start_ns=None, end_ns=None):
    """両側にデータがある端点範囲だけを照合し、境界外を欠測に数えない。"""
    if not mission or not sidecar:
        return dict(status='missing_topic', mission_total=len(mission), sidecar_total=len(sidecar))
    low = max(mission[0][0], sidecar[0][0])
    high = min(mission[-1][0], sidecar[-1][0])
    if start_ns is not None:
        low = max(low, start_ns)
    if end_ns is not None:
        high = min(high, end_ns)
    left = [row for row in mission if low <= row[0] <= high]
    right = [row for row in sidecar if low <= row[0] <= high]
    if high < low or not left or not right:
        return dict(status='no_common_interval', mission_total=len(mission), sidecar_total=len(sidecar))
    # 同じheaderでも値が異なる場合を一致扱いせず、重複payloadの件数も保つ。
    left_counts = Counter((stamp, digest) for stamp, _, digest in left)
    right_counts = Counter((stamp, digest) for stamp, _, digest in right)
    only_left, only_right = left_counts - right_counts, right_counts - left_counts
    return dict(status='ok', start_header_ns=low, end_header_ns=high,
                mission=metrics(left), sidecar=metrics(right),
                only_mission=sum(only_left.values()), only_sidecar=sum(only_right.values()),
                only_mission_header_ns=[stamp for (stamp, _), count in only_left.items()
                                        for _ in range(count)],
                only_sidecar_header_ns=[stamp for (stamp, _), count in only_right.items()
                                        for _ in range(count)])


def stamp_seconds(ns):
    """既存probeのfloat形式と同じsec+nsec/1e9でheaderを表す。"""
    sec, nsec = divmod(ns, 1_000_000_000)
    return sec + nsec / 1e9


def add_probe_evidence(results, path, mission=None, sidecar=None):
    """独立購読のheader到達を照合する。probeはpayloadの一致までは証明しない。"""
    seen = {topic: Counter() for topic in TOPICS}
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            if row['topic'] in seen:
                seen[row['topic']][float(row['header_s'])] += 1
    for topic, result in results.items():
        if result['status'] != 'ok':
            continue
        missing = Counter(stamp_seconds(ns) for ns in result['only_sidecar_header_ns'])
        result['only_sidecar_seen_probe'] = sum((missing & seen[topic]).values())
        if mission is not None and sidecar is not None:
            low, high = result['start_header_ns'], result['end_header_ns']
            probe_window = Counter({stamp: count for stamp, count in seen[topic].items()
                                    if stamp_seconds(low) <= stamp <= stamp_seconds(high)})
            result['probe_header_count'] = sum(probe_window.values())
            for name, dataset in (('mission', mission), ('sidecar', sidecar)):
                lane = Counter(stamp_seconds(stamp) for stamp, _, _ in dataset[topic]
                               if low <= stamp <= high)
                # 独立probeにも固有欠測があり得るので、双方にないheaderを別々に数える。
                # ここはheader到達の比較であり、SHA256の内容一致とは意味が異なる。
                result['probe_headers_not_' + name] = sum((probe_window - lane).values())
                result[name + '_headers_not_probe'] = sum((lane - probe_window).values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission-bag', type=Path, required=True)
    parser.add_argument('--imu-bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--probe-csv', type=Path)
    parser.add_argument('--start-header-s', type=float, help='共通区間の下限[epoch秒]')
    parser.add_argument('--end-header-s', type=float, help='共通区間の上限[epoch秒]')
    args = parser.parse_args()
    for value in (args.start_header_s, args.end_header_s):
        if value is not None and not math.isfinite(value):
            parser.error('header範囲は有限の秒数で指定してください')
    if (args.start_header_s is not None and args.end_header_s is not None
            and args.end_header_s <= args.start_header_s):
        parser.error('終了時刻は開始時刻より後にしてください')
    if args.output.exists():
        parser.error('既存の比較結果は上書きしません')
    mission = read_imu(args.mission_bag)
    sidecar = read_imu(args.imu_bag)
    start = round(args.start_header_s * 1e9) if args.start_header_s is not None else None
    end = round(args.end_header_s * 1e9) if args.end_header_s is not None else None
    results = {topic: compare_topic(mission[topic], sidecar[topic], start, end) for topic in TOPICS}
    if args.probe_csv:
        add_probe_evidence(results, args.probe_csv, mission, sidecar)
    summary = dict(mission_bag=str(args.mission_bag), imu_bag=str(args.imu_bag), topics=results)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2) + '\n')
    # 欠測した全時刻はJSONに保持し、consoleには大量の時刻配列を出さない。
    print(json.dumps({topic: {key: value for key, value in result.items()
                             if not key.endswith('_header_ns')}
                      for topic, result in results.items()}, indent=2), flush=True)


if __name__ == '__main__':
    main()
