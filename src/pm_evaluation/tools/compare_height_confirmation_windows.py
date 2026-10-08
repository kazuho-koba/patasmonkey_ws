#!/usr/bin/env python3
"""同一trackに限ってN変更による初回確認の追加待ち時間を比較する。"""
import argparse
import csv
import gzip
import json
from pathlib import Path

import numpy as np


def confirmations(folder):
    """全候補行をstreamし、初回確認行だけ保持する。欠測を補わない。"""
    path = folder/'tracks.csv'
    opener = open
    if not path.exists():
        path = folder/'tracks.csv.gz'; opener = gzip.open
    result = {}
    with opener(path, 'rt') as stream:
        for row in csv.DictReader(stream):
            if row['first_confirmation'] == 'True':
                result[int(row['track_id'])] = (int(row['stamp_ns']), int(row['first_positive_stamp']),
                                               int(row['cell_x']), int(row['cell_y']))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('long_window_directory'); parser.add_argument('--short-window-directory', required=True)
    args = parser.parse_args(); output = Path(args.long_window_directory)
    long = confirmations(output); short = confirmations(Path(args.short_window_directory))
    differences = []
    for track, row in long.items():
        if track not in short or short[track][1:] != row[1:]:
            raise ValueError('初回positive／セル／track対応が不一致です')
        extra = (row[0]-short[track][0])/1e9
        if extra < 0:
            raise ValueError('長い窓が先に確認されています')
        differences.append(extra)
    values = np.asarray(differences)
    result = dict(short_confirmed_tracks=len(short), long_confirmed_tracks=len(long),
        paired_tracks=len(values), additional_confirmation_wait_seconds=dict(
            min=float(values.min()), mean=float(values.mean()), median=float(np.median(values)),
            p95=float(np.percentile(values,95)), max=float(values.max())))
    (output/'paired_confirmation_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
