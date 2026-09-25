#!/usr/bin/env python3
"""通過前hazard評価のCSVを、実走行軌跡上の色と距離順へ可視化する。"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt
import numpy as np

# GUIを持たないコンテナでは日本語fontが未導入のことがあり、Matplotlibが警告だけ
# 出して日本語glyphのないfontへ置換するとPNGに豆腐が残る。利用可能性を明示確認し、
# fontがない環境では同じ情報を英語labelで描いて、読めない画像を作らない。
try:
    font_manager.findfont("Noto Sans CJK JP", fallback_to_default=False)
    plt.rcParams["font.family"] = "Noto Sans CJK JP"
    USE_JAPANESE_LABELS = True
except ValueError:
    plt.rcParams["font.family"] = "DejaVu Sans"
    USE_JAPANESE_LABELS = False


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

    # Fontの有無で表現だけを切り替え、データ系列や色の意味は常に同じに保つ。
    if USE_JAPANESE_LABELS:
        label_not_black = "中心は黒ではない"
        label_black = "中心が黒"
        label_unknown = "中心が未観測/先読み不可"
        label_black_fraction = "占有範囲内の黒セル率"
        label_coverage = "占有範囲の観測率"
        title_path = "実走行軌跡：2 m手前の中心hazard"
        title_footprint = "通過予定の占有範囲：黒セルと観測率"
        xlabel_distance = "累積走行距離 [m]"
        ylabel_fraction = "割合"
    else:
        label_not_black = "Center not black"
        label_black = "Center black"
        label_unknown = "Unobserved / no lookahead"
        label_black_fraction = "Black cells in footprint"
        label_coverage = "Footprint observation coverage"
        title_path = "Traversed path: center hazard 2 m ahead"
        title_footprint = "Footprint ahead: black cells and coverage"
        xlabel_distance = "Cumulative travel distance [m]"
        ylabel_fraction = "Fraction"

    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    axes[0].plot(x, y, color="lightgray", linewidth=1, zorder=0)
    axes[0].scatter(x[observed & ~black], y[observed & ~black], s=13,
                    color="#287b3f", label=label_not_black)
    axes[0].scatter(x[black], y[black], s=20, color="#d62728",
                    label=label_black)
    axes[0].scatter(x[~observed], y[~observed], s=10, color="#888888",
                    label=label_unknown)
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_xlabel("odom x [m]")
    axes[0].set_ylabel("odom y [m]")
    axes[0].set_title(title_path)
    axes[0].legend(fontsize=8, loc="lower right")

    # 欠測を線でつなぐと、観測していない区間に値が存在するように見えるため点表示する。
    axes[1].scatter(distance, fraction, color="#d62728", s=10,
                    label=label_black_fraction)
    axes[1].plot(distance, coverage, color="#1f77b4", linewidth=1,
                 alpha=0.75, label=label_coverage)
    axes[1].scatter(distance[black], np.ones(black.sum()), s=12,
                    color="black", label=label_black)
    axes[1].set_xlim(0, distance[-1])
    axes[1].set_ylim(-0.03, 1.05)
    axes[1].set_xlabel(xlabel_distance)
    axes[1].set_ylabel(ylabel_fraction)
    axes[1].set_title(title_footprint)
    axes[1].legend(fontsize=8, loc="upper right")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    print("saved", args.output)


if __name__ == "__main__":
    main()
