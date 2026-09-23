#!/usr/bin/env python3
"""走行経路中心でblackだった絶対odom-cellを時系列forensic用CSVにする。

通常のforensic抽出CSVはmap timestampもkeyにするため、後段の再生ではその時刻だけを
記録する。このツールはmap timestampを空にしたtarget CSVを作り、同じ5 cm odom-cellが
rolling map内で観測される各時刻のground candidate/fusion状態を追跡する。
"""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path_samples", type=Path,
                        help="terrain replayで生成したpath_samples.csv")
    parser.add_argument("output_csv", type=Path,
                        help="空map_stamp_nsのspatial target CSV出力先")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="hazard grid resolution [m]。現在の既定値は5 cm")
    args = parser.parse_args()
    if args.resolution <= 0.0:
        parser.error("--resolution must be positive")

    # 同じ走行cellが複数path sampleに出てもspatial targetは一度だけ記録する。
    targets = defaultdict(int)
    with args.path_samples.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if row.get("center_hazard") != "100":
                continue
            if not row.get("lookahead_map_time_ns"):
                continue
            x = float(row["x_odom_m"])
            y = float(row["y_odom_m"])
            cell_x = math.floor(x / args.resolution)
            cell_y = math.floor(y / args.resolution)
            targets[(cell_x, cell_y)] += 1

    # map_stamp_nsを空にするとmapperは対象cellを全map時刻で記録する。
    # source_sample_countを残し、どのcellが複数回の走行sampleに対応したか確認できる。
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    columns = ("map_stamp_ns", "odom_cell_x", "odom_cell_y", "source_sample_count")
    with args.output_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for (cell_x, cell_y), sample_count in sorted(targets.items()):
            writer.writerow({
                "map_stamp_ns": "",
                "odom_cell_x": cell_x,
                "odom_cell_y": cell_y,
                "source_sample_count": sample_count,
            })

    print("black path samples mapped:", sum(targets.values()))
    print("unique spatial cells:", len(targets))
    print("saved:", args.output_csv)


if __name__ == "__main__":
    main()
