#!/usr/bin/env python3
"""通過前hazard評価のCSVを、実走行軌跡上の色と距離順へ可視化する。"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = "Noto Sans CJK JP"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = list(csv.DictReader(args.csv.open(encoding="utf-8")))
    x = np.array([float(r["x_odom_m"]) for r in rows])
    y = np.array([float(r["y_odom_m"]) for r in rows])
    distance = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]

    # 観測済み中心の黒/非黒と未観測を分ける。灰色には「2 m先で見えなかった」
    # 地点も含むため、走行可能と誤読させない。
    center = np.array([int(r["center_hazard"]) if r["center_hazard"] else -1
                       for r in rows])
    observed = center >= 0
    black = center >= 100
    fraction = np.array([float(r["black_fraction_observed"])
                         if r["black_fraction_observed"] else np.nan for r in rows])
    coverage = np.array([int(r["observed_cells"]) / max(1, int(r["footprint_cells"]))
                         for r in rows])

    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    axes[0].plot(x, y, color="lightgray", linewidth=1, zorder=0)
    axes[0].scatter(x[observed & ~black], y[observed & ~black], s=13,
                    color="#287b3f", label="中心は黒ではない")
    axes[0].scatter(x[black], y[black], s=20, color="#d62728",
                    label="中心が黒")
    axes[0].scatter(x[~observed], y[~observed], s=10, color="#888888",
                    label="中心が未観測/先読み不可")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_xlabel("odom x [m]")
    axes[0].set_ylabel("odom y [m]")
    axes[0].set_title("実走行軌跡：2 m手前の中心hazard")
    axes[0].legend(fontsize=8, loc="lower right")

    # 欠測を線でつなぐと、観測していない区間に値が存在するように見えるため点表示する。
    axes[1].scatter(distance, fraction, color="#d62728", s=10,
                    label="占有範囲内の黒セル率")
    axes[1].plot(distance, coverage, color="#1f77b4", linewidth=1,
                 alpha=0.75, label="占有範囲の観測率")
    axes[1].scatter(distance[black], np.ones(black.sum()), s=12,
                    color="black", label="中心が黒")
    axes[1].set_xlim(0, distance[-1])
    axes[1].set_ylim(-0.03, 1.05)
    axes[1].set_xlabel("累積走行距離 [m]")
    axes[1].set_ylabel("割合")
    axes[1].set_title("通過予定の占有範囲：黒セルと観測率")
    axes[1].legend(fontsize=8, loc="upper right")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    print("saved", args.output)


if __name__ == "__main__":
    main()
