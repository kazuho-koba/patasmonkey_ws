#!/usr/bin/env python3
"""複数試験の全topicと独立プローブ照合をCSVへ並べる。

入力はaudit_bag_delivery.pyのsummary.json。件数の単純差は起動時間差を含むため、
header周期・最大間隔と、共通区間で到達確認できたmessageの未記録割合を出力する。
プローブにも欠測があるので、この割合を真のpublisher欠測率とは解釈しない。
"""
import argparse
import csv
import json
from pathlib import Path


def comparison_rows(trials):
    """試験ごとにtopicを展開し、未観測の指標は空欄に保つ。"""
    rows = []
    for label, summary in trials:
        probes = summary.get('independent_imu_probe') or {}
        for topic, values in sorted(summary['topics'].items()):
            probe = probes.get(topic, {})
            denominator = probe.get('probe_count', 0)
            missing = probe.get('probe_only')
            rows.append(dict(
                trial=label, topic=topic, count=values['count'],
                header_hz=values.get('header_active_hz'),
                max_header_gap_s=values.get('max_header_gap_s'),
                max_record_gap_s=values.get('max_record_gap_s'),
                probe_count=denominator or None, probe_only=missing,
                observed_missing_percent=(100*missing/denominator
                                          if denominator and missing is not None else None)))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', action='append', required=True, metavar='LABEL=SUMMARY_JSON')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('既存の比較結果を上書きしません')
    trials = []
    for item in args.trial:
        label, separator, filename = item.partition('=')
        if not separator or not label:
            parser.error('--trialはLABEL=SUMMARY_JSONで指定してください')
        trials.append((label, json.loads(Path(filename).read_text())))
    rows = comparison_rows(trials)
    if not rows:
        parser.error('比較可能なtopicがありません')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == '__main__':
    main()
