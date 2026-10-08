#!/usr/bin/env python3
"""保存CSVから疎な高点の反復支持を集計し、summaryと件数を照合する。"""
import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sequence_directory')
    args = parser.parse_args(); folder = Path(args.sequence_directory)
    summary = json.loads((folder/'summary.json').read_text())
    tracks = defaultdict(Counter); observations = Counter(); high = Counter(); matched_high = Counter()
    with (folder/'tracks.csv').open() as stream:
        for row in csv.DictReader(stream):
            kind = row['kind']; observations[kind] += 1
            t = tracks[int(row['track_id'])]; t['observations'] += 1
            if row['high_hit'] == 'True':
                high[kind] += 1; t['positive'] += 1
                matched_high[kind] += row['matched'] == 'True'
                t['sparse_positive'] += kind == 'sparse'
            t['confirmed'] += row['first_confirmation'] == 'True'
    # 間違った実験directoryや途中CSVを有効な完了試験と扱わない。
    expected = summary['matching']['new']+summary['matching']['matched']
    if sum(observations.values()) != expected or len(tracks) != summary['matching']['new']:
        raise ValueError('CSV件数とsummaryが不一致です')
    positive = [t for t in tracks.values() if t['positive']]
    sparse = [t for t in tracks.values() if t['sparse_positive']]
    result = dict(observations_by_kind=dict(observations), high_observations_by_kind=dict(high),
        matched_high_observations_by_kind=dict(matched_high), positive_tracks=len(positive),
        positive_tracks_once=sum(t['positive'] == 1 for t in positive),
        positive_tracks_three_or_more=sum(t['positive'] >= 3 for t in positive),
        confirmed_tracks=sum(t['confirmed'] > 0 for t in tracks.values()),
        sparse_positive_tracks=len(sparse), sparse_positive_tracks_once=sum(t['positive']==1 for t in sparse),
        sparse_positive_tracks_confirmed=sum(t['confirmed'] > 0 for t in sparse))
    # N1と保守側は全footprint行の値まで一致することを独立に検査する。
    rows = list(csv.DictReader((folder/'footprints.csv').open()))
    modes = {m: {r['stamp_ns']: {k:v for k,v in r.items() if k != 'mode'}
                 for r in rows if r['mode'] == m} for m in summary['results']}
    conservative_mode = next(mode for mode in modes if mode.endswith('_conservative'))
    if modes['N1'] != modes[conservative_mode]:
        raise ValueError('N1と保守側の結果が異なります')
    result['n1_conservative_identical_all_footprints'] = True
    (folder/'sequence_audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
