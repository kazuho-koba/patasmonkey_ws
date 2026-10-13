"""async writer CSVの圧縮前CDR帯域と滞留回復を解析する（ROS不要）。

writer起動からのelapsed_sを使用し、bag/header時計とは同一視しない。
1 Hz sample間の平均なので瞬間peakや停滞中の最大量はYAML統計も併用する。
圧縮後SSD帯域・電源断時の永続化・DDSより前の欠測はこのCSVでは分からない。
"""
import argparse
import csv
import json
import math
from pathlib import Path


def summarize(rows):
    """累積byteの隣接差分を計算する。旧CSVを流入0と誤認せず拒否する。"""
    required = ('elapsed_s', 'accepted_bytes', 'written_bytes', 'pending_bytes', 'pending_items')
    if len(rows) < 2 or any(key not in row for row in rows for key in required):
        raise ValueError('byte統計付きCSVの2行以上が必要です（旧CSVは未対応）')
    samples = [{key: float(row[key]) for key in required} for row in rows]
    if any(not math.isfinite(value) or value < 0 for row in samples for value in row.values()):
        raise ValueError('統計には非負の有限数が必要です')
    intervals = []
    for before, after in zip(samples, samples[1:]):
        seconds = after['elapsed_s'] - before['elapsed_s']
        accepted = after['accepted_bytes'] - before['accepted_bytes']
        written = after['written_bytes'] - before['written_bytes']
        if seconds <= 0 or accepted < 0 or written < 0:
            raise ValueError('時計は増加、累積byteは非減少である必要があります')
        intervals.append({
            'start_s': before['elapsed_s'], 'end_s': after['elapsed_s'],
            'accepted_bytes_per_s': accepted / seconds,
            'written_bytes_per_s': written / seconds,
            'pending_bytes': int(after['pending_bytes']),
            'pending_items': int(after['pending_items']),
        })
    # 差分帯域が一時的に保存<受信でも、後段でpendingが減れば回復している。
    # 最終0はflush完了の指標であり、途中の容量超過が無かったことは保証しない。
    return {
        'sample_count': len(samples),
        'sampled_peak_pending_bytes': int(max(row['pending_bytes'] for row in samples)),
        'sampled_peak_pending_items': int(max(row['pending_items'] for row in samples)),
        'final_pending_bytes': int(samples[-1]['pending_bytes']),
        'final_pending_items': int(samples[-1]['pending_items']),
        'final_accepted_bytes': int(samples[-1]['accepted_bytes']),
        'final_written_bytes': int(samples[-1]['written_bytes']),
        'intervals': intervals,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', required=True, type=Path)
    args = parser.parse_args()
    try:
        with args.csv.open(newline='') as stream:
            result = summarize(list(csv.DictReader(stream)))
    except (ValueError, OSError) as error:
        parser.error(str(error))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
