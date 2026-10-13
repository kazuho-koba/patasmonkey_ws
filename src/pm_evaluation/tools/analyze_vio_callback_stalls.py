#!/usr/bin/env python3
"""停止済みbagのIMU受信間隔と、VIO自身のcallback停滞ログを同じheaderで照合する。

独立購読は作らない。recorderの受信時計はdriverのpublish時計ではないので、
一致しても共通の停滞箇所を断定せず、不一致でもDDS・OS待ちを区別できない。
"""
import argparse
from bisect import bisect_left, bisect_right
import csv
import json
from pathlib import Path
import re


def parse_delays(text):
    """headerが連続したcallback停滞だけを抽出する。値の単位はnative logのms/秒。

    欠測ログには異なる意味があるため混ぜない。再現時はIMU_INPUT_GAPと
    IMU_GAP_CONTEXTも併せて原文を読む。未知の追加fieldは無視する。
    """
    result = []
    for line in text.splitlines():
        if '[IMU_CALLBACK_DELAY]' not in line:
            continue
        values = dict(re.findall(r'(\w+)=([-+\d.eE]+)', line))
        required = ('callback_gap_ms', 'header_delta_ms', 'previous_feed_ms', 'previous_odometry_ms', 'stamp')
        if not all(key in values for key in required):
            continue
        result.append({key: float(values[key]) for key in required})
    return result


def nearest_stamp(sorted_stamps, target, tolerance_ns=2000):
    """native doubleと小数3桁のdelta[ms]の丸めを考慮し、2 us以内だけ対応付ける。

    125 Hzの別サンプルを誤って対応付けない。対応がなければ保存欠落と断定せずNone。
    """
    index = bisect_left(sorted_stamps, target)
    candidates = sorted_stamps[max(0, index-1):index+1]
    if not candidates:
        return None
    best = min(candidates, key=lambda stamp: abs(stamp-target))
    return best if abs(best-target) <= tolerance_ns else None


def parse_input_gaps(text):
    """受信header欠測・逆行の原文を抽出する。callback開始空白とは別に扱う。"""
    result = []
    for line in text.splitlines():
        if '[IMU_INPUT_GAP]' not in line:
            continue
        values = dict(re.findall(r'(\w+)=([-+\d.eE]+)', line))
        required = ('previous', 'current', 'delta', 'receipt')
        if all(key in values for key in required):
            result.append({key:float(values[key]) for key in required})
    return result


def match_input_gaps(gaps, records, driver_records):
    """VIOの欠測区間内に保存されたIMUと、driver時計の進み方を数える。

    同じheaderの最初の受信に固定し、両端を除いた内部サンプルだけを数える。
    driver側で通常の測定周期でpublishされたのか、まとめて出たのかの証拠を残す。
    これだけではVIOのdeserialize・executor・publish待ちのどれかを断定しない。
    """
    result = []
    source_stamps = sorted(records)
    driver_stamps = sorted(driver_records)
    for gap in gaps:
        row = dict(gap)
        for label, stamps, times in (('source_record', source_stamps, records),
                                     ('driver_post_publish', driver_stamps, driver_records)):
            first = nearest_stamp(stamps, round(gap['previous']*1e9))
            last = nearest_stamp(stamps, round(gap['current']*1e9))
            row[label+'_endpoint_span_ms'] = None
            row[label+'_internal_count'] = None
            row[label+'_internal_span_ms'] = None
            if first is None or last is None or first >= last:
                continue
            row[label+'_endpoint_span_ms'] = (times[last]-times[first])/1e6
            internal = stamps[bisect_right(stamps, first):bisect_left(stamps, last)]
            row[label+'_internal_count'] = len(internal)
            if len(internal) >= 2:
                row[label+'_internal_span_ms'] = (times[internal[-1]]-times[internal[0]])/1e6
        result.append(row)
    return result


def match_record_intervals(events, records, driver_records=None):
    """同じ2サンプルのVIO callback間隔とbag受信間隔を並べる。

    recordsはheader[整数ns]→recorder受信時刻[整数ns]。重複headerは呼出側で
    最初の受信に固定する。bag intervalが短ければVIO側だけの遅延候補だが、
    recorderにも独自の待ちがあり、単独で因果関係を決められない。
    """
    stamps = sorted(records)
    driver_records = driver_records or {}
    driver_stamps = sorted(driver_records)
    result = []
    for event in events:
        current = nearest_stamp(stamps, round(event['stamp']*1e9))
        previous = nearest_stamp(stamps, round((event['stamp']-event['header_delta_ms']/1000)*1e9))
        row = dict(event, source_record_gap_ms=None, source_header_gap_ms=None, interval_difference_ms=None,
                   driver_post_publish_gap_ms=None)
        if current is not None and previous is not None:
            row['source_record_gap_ms'] = (records[current]-records[previous])/1e6
            row['source_header_gap_ms'] = (current-previous)/1e6
            row['interval_difference_ms'] = event['callback_gap_ms']-row['source_record_gap_ms']
        driver_current = nearest_stamp(driver_stamps, round(event['stamp']*1e9))
        driver_previous = nearest_stamp(driver_stamps, round((event['stamp']-event['header_delta_ms']/1000)*1e9))
        if driver_current is not None and driver_previous is not None:
            row['driver_post_publish_gap_ms'] = (driver_records[driver_current]-driver_records[driver_previous])/1e6
        result.append(row)
    return result


def largest_record_gap_evidence(records, driver_records, limit=5):
    """bag受信の最大空白を、同じheaderのdriver時計と照合する。

    重複headerの最初の受信を使う。受信順に並べ、header逆行も符号付きで残す。
    driver診断が欠けた場合はNoneで、driver停滞がなかったとは判断しない。
    """
    ordered = sorted(records.items(), key=lambda item: item[1])
    pairs = sorted(zip(ordered, ordered[1:]), key=lambda pair: pair[1][1]-pair[0][1], reverse=True)[:limit]
    driver_stamps = sorted(driver_records)
    result = []
    for (previous, previous_receipt), (current, current_receipt) in pairs:
        first = nearest_stamp(driver_stamps, previous)
        last = nearest_stamp(driver_stamps, current)
        driver_gap = None if first is None or last is None else (driver_records[last]-driver_records[first])/1e6
        result.append(dict(previous_header_ns=previous, current_header_ns=current,
                           record_gap_ms=(current_receipt-previous_receipt)/1e6,
                           header_gap_ms=(current-previous)/1e6,
                           driver_post_publish_gap_ms=driver_gap))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--core-log', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--topic', default='/oak/imu/data')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('既存解析を上書きしません。新しいoutputディレクトリを指定してください。')
    # ROS依存はCLI実行時だけ読み込む。純粋な照合関数はホストでも単体試験できる。
    from mcap.reader import make_reader
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.serialization import deserialize_message
    from audit_bag_delivery import header_stamp
    from pm_evaluation.cli.analyze_bag_frequencies import load_bag_metadata, find_mcap_files
    metadata = load_bag_metadata(args.bag/'metadata.yaml')
    records = {}
    driver_records = {}
    for filename in find_mcap_files(metadata):
        with filename.open('rb') as stream:
            # 巨大な画像をdeserializeせず、対象IMUのCDR Headerだけを読む。
            for schema, channel, message in make_reader(stream).iter_messages(topics=[args.topic, '/oak/diagnostics/imu_packet']):
                stamp = header_stamp(message.data, schema.name)
                if stamp is not None:
                    if channel.topic == args.topic:
                        records.setdefault(stamp, message.log_time)
                    else:
                        # 既存診断のhost_receive_time_nsは、このdriverではIMU publish後に
                        # 時計を読む値。USB受信・publish開始・完了そのものの時刻ではない。
                        diagnostic = deserialize_message(message.data, DiagnosticArray)
                        for status in diagnostic.status:
                            values = {item.key:item.value for item in status.values}
                            try:
                                driver_records.setdefault(stamp, int(values['host_receive_time_ns']))
                            except (KeyError, ValueError):
                                pass
    text = args.core_log.read_text()
    rows = match_record_intervals(parse_delays(text), records, driver_records)
    input_gaps = match_input_gaps(parse_input_gaps(text), records, driver_records)
    args.output.mkdir(parents=True, exist_ok=True)
    fields = ['stamp', 'callback_gap_ms', 'header_delta_ms', 'previous_feed_ms', 'previous_odometry_ms',
              'source_record_gap_ms', 'source_header_gap_ms', 'interval_difference_ms', 'driver_post_publish_gap_ms']
    with (args.output/'callback_delays.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    gap_fields = ['previous', 'current', 'delta', 'receipt']
    gap_fields += [label+suffix for label in ('source_record', 'driver_post_publish')
                   for suffix in ('_endpoint_span_ms', '_internal_count', '_internal_span_ms')]
    with (args.output/'input_gaps.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=gap_fields)
        writer.writeheader()
        writer.writerows(input_gaps)
    summary = dict(events=len(rows), matched_events=sum(row['source_record_gap_ms'] is not None for row in rows),
                   topic=args.topic, bag=str(args.bag), all_publish_retention_percent=None,
                   input_gaps=input_gaps,
                   largest_record_gap_evidence=largest_record_gap_evidence(records, driver_records),
                   interpretation='受信間隔の比較であり、publish時計・SSD書込み完了時計ではない。原因の断定には使わない。')
    (args.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    main()
