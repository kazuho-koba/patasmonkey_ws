#!/usr/bin/env python3
"""Overlay a heading-initialized local odometry CSV on GNSS and OSM.

Unlike the legacy trajectory plotter, this tool intentionally performs no
heading rotation. A calibrated initial yaw should already express local x/y in
the ENU frame; the only permitted alignment is a translation to the nearest
GNSS point at the local-odometry start time. This makes a wrong heading
calibration visible rather than hiding it with a plotting offset.
"""

import argparse
import csv
import json
import math
from pathlib import Path

from pm_evaluation.cli.plot_bag_trajectories import (
    build_time_index,
    choose_rtk_status_stream,
    compute_gnss_reference_bounds,
    convert_fixes_to_local_xy,
    nearest_sample,
    plot_gnss,
    read_bag_data,
    save_figure,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Source bag containing /fix")
    parser.add_argument("odometry_csv", type=Path,
                        help="composed_local.csv from replay_separated_localization")
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def read_csv(path):
    rows = []
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            rows.append({key: float(row[key]) for key in ("t", "x", "y")})
    return rows


def translate_to_gnss(odometry, gnss):
    if not odometry or not gnss:
        return []
    gnss_ns = build_time_index(gnss)
    # CSV timestamp is seconds while the shared plotting helpers use ns.
    anchor = nearest_sample(gnss, gnss_ns, int(odometry[0]["t"] * 1e9))
    dx = anchor["x"] - odometry[0]["x"]
    dy = anchor["y"] - odometry[0]["y"]
    return [{"t": row["t"], "x": row["x"] + dx, "y": row["y"] + dy}
            for row in odometry]


def error_summary(trajectory, gnss):
    if not trajectory or not gnss:
        return {"matched_points": 0}
    timestamps = [row["t"] for row in trajectory]
    errors = []
    for point in gnss:
        match = nearest_sample(trajectory, timestamps, point["t"] * 1e-9)
        errors.append(math.hypot(match["x"] - point["x"], match["y"] - point["y"]))
    return {
        "matched_points": len(errors),
        "rmse_m": math.sqrt(sum(value * value for value in errors) / len(errors)),
        "median_error_m": sorted(errors)[len(errors) // 2],
        "max_error_m": max(errors),
    }


def main():
    args = parse_args()
    if not args.bag.is_dir() or not args.odometry_csv.is_file():
        raise SystemExit("Input bag directory or odometry CSV does not exist")
    if args.output_dir.exists():
        raise SystemExit("Refusing to overwrite output directory: {}".format(args.output_dir))
    args.output_dir.mkdir(parents=True)

    data = read_bag_data(args.bag)
    gnss, mercator_origin = convert_fixes_to_local_xy(
        data["fixes"], choose_rtk_status_stream(data)
    )
    raw = read_csv(args.odometry_csv)
    trajectory = translate_to_gnss(raw, gnss)
    if not gnss or not trajectory:
        raise SystemExit("Both GNSS fixes and initialized odometry are required")
    bounds = compute_gnss_reference_bounds(gnss)
    summary = error_summary(trajectory, gnss)

    def plot(ax):
        plot_gnss(ax, gnss)
        ax.plot([row["x"] for row in trajectory],
                [row["y"] for row in trajectory], color="tab:blue",
                linewidth=1.5, label="heading-initialized local odometry", zorder=3)
        ax.scatter([trajectory[0]["x"]], [trajectory[0]["y"]],
                   color="tab:green", label="odometry start", zorder=4)
        ax.legend(loc="best")

    output_png = args.output_dir / "initialized_odometry_gnss_osm_overlay.png"
    save_figure(
        output_png,
        "Heading-initialized local odometry vs GNSS (no plot rotation)",
        bounds,
        mercator_origin,
        plot,
    )
    payload = {
        "input_bag": str(args.bag),
        "input_odometry_csv": str(args.odometry_csv),
        "alignment": "translation only; no heading rotation or yaw offset applied",
        **summary,
    }
    (args.output_dir / "initialized_odometry_gnss_overlay_summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
