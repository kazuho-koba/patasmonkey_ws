#!/usr/bin/env python3
"""Estimate the plotting heading offset from GNSS position and velocity."""

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyproj import CRS, Transformer


def load(path):
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return data[np.argsort(data[:, 0], kind="stable")]


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s], [s, c]])


def solve_rotation(source, target, weights=None):
    if weights is None:
        weights = np.ones(len(source))
    dot = np.sum(weights * np.sum(source * target, axis=1))
    cross = np.sum(weights * (source[:, 0] * target[:, 1] -
                              source[:, 1] * target[:, 0]))
    return math.atan2(cross, dot)


def angle_residual(source, target, angle):
    predicted = source @ rotation(angle).T
    dot = np.sum(predicted * target, axis=1)
    cross = predicted[:, 0] * target[:, 1] - predicted[:, 1] * target[:, 0]
    return np.degrees(np.arctan2(cross, dot))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    args = parser.parse_args()
    out = args.results.resolve()

    odom = load(out / "wheel_gyro.csv")
    imu = load(out / "imu.csv")
    fixes = load(out / "fix.csv")
    velocity = load(out / "fix_velocity.csv")
    summary = json.loads((out / "summary.json").read_text())
    start, end = summary["common_start"], summary["common_end"]

    imu_yaw = np.interp(start, imu[:, 0], np.unwrap(imu[:, 1]))
    odom_yaw = np.interp(start, odom[:, 0], np.unwrap(odom[:, 4]))

    # GNSS positions in a metric local east/north projection.
    lon0, lat0 = fixes[0, 1:3]
    local_crs = CRS.from_proj4(
        f"+proj=aeqd +lat_0={lat0} +lon_0={lon0} +datum=WGS84 +units=m")
    transformer = Transformer.from_crs(4326, local_crs, always_xy=True)
    east, north = transformer.transform(fixes[:, 1], fixes[:, 2])
    gnss_xy = np.column_stack([east, north])

    # Local EKF velocity expressed in its odom frame. Odometry twist is in the
    # child/body frame, so rotate vx/vy by the estimated yaw first.
    vt = velocity[:, 0]
    valid_time = (vt >= start) & (vt <= end)
    vt = vt[valid_time]
    gv = velocity[valid_time, 1:3]
    gvar = velocity[valid_time, 4:6]
    oy = np.interp(vt, odom[:, 0], np.unwrap(odom[:, 4]))
    ovx = np.interp(vt, odom[:, 0], odom[:, 5])
    ovy = np.interp(vt, odom[:, 0], odom[:, 6])
    lv = np.column_stack([ovx*np.cos(oy)-ovy*np.sin(oy),
                          ovx*np.sin(oy)+ovy*np.cos(oy)])

    velocity_solutions = []
    primary = None
    for min_speed in [0.4, 0.6, 0.8]:
        for max_sigma in [0.6, 0.8, 1.0]:
            gs = np.linalg.norm(gv, axis=1)
            ls = np.linalg.norm(lv, axis=1)
            sigma = np.sqrt(np.maximum(gvar[:, 0], gvar[:, 1]))
            mask = (gs >= min_speed) & (ls >= min_speed) & (sigma <= max_sigma)
            if np.count_nonzero(mask) < 5:
                continue
            weights = 1.0 / np.maximum(gvar[mask].sum(axis=1), 1e-6)
            angle = solve_rotation(lv[mask], gv[mask], weights)
            offset = wrap_deg(math.degrees(angle + odom_yaw - imu_yaw))
            residual = angle_residual(lv[mask], gv[mask], angle)
            item = {
                "min_speed_mps": min_speed,
                "max_component_sigma_mps": max_sigma,
                "samples": int(np.count_nonzero(mask)),
                "local_to_enu_rotation_deg": wrap_deg(math.degrees(angle)),
                "plot_offset_added_to_published_wit_yaw_deg": offset,
                "course_residual_median_abs_deg": float(np.median(np.abs(residual))),
                "course_residual_p90_abs_deg": float(np.percentile(np.abs(residual), 90)),
            }
            velocity_solutions.append(item)
            if min_speed == 0.6 and max_sigma == 1.0:
                primary = (item, mask, weights, angle)
    if primary is None:
        raise RuntimeError("Not enough usable GNSS velocity samples")

    # Position-increment solutions reduce sensitivity to an arbitrary GNSS
    # translation. Different lags expose sensitivity to the noisy 1 Hz fixes.
    ft = fixes[:, 0]
    common_fix = (ft >= start) & (ft <= end) & (fixes[:, 3] >= 0)
    ft, gxy = ft[common_fix], gnss_xy[common_fix]
    lxy = np.column_stack([
        np.interp(ft, odom[:, 0], odom[:, 1]),
        np.interp(ft, odom[:, 0], odom[:, 2]),
    ])
    position_solutions = []
    increment_sets = {}
    for lag in [5, 10, 20, 30]:
        source, target = lxy[lag:] - lxy[:-lag], gxy[lag:] - gxy[:-lag]
        mask = (np.linalg.norm(source, axis=1) >= 2.0) & (
            np.linalg.norm(target, axis=1) >= 2.0)
        source, target = source[mask], target[mask]
        if len(source) < 3:
            continue
        angle = solve_rotation(source, target)
        offset = wrap_deg(math.degrees(angle + odom_yaw - imu_yaw))
        residual = angle_residual(source, target, angle)
        position_solutions.append({
            "lag_s_approximately": lag,
            "samples": len(source),
            "local_to_enu_rotation_deg": wrap_deg(math.degrees(angle)),
            "plot_offset_added_to_published_wit_yaw_deg": offset,
            "course_residual_median_abs_deg": float(np.median(np.abs(residual))),
            "course_residual_p90_abs_deg": float(np.percentile(np.abs(residual), 90)),
        })
        increment_sets[lag] = (source, target)

    # Whole-trajectory Procrustes is included as a deliberately GNSS-drift-
    # sensitive cross-check, not as the preferred result.
    lc, gc = lxy-lxy.mean(axis=0), gxy-gxy.mean(axis=0)
    procrustes_angle = solve_rotation(lc, gc)
    procrustes_offset = wrap_deg(math.degrees(
        procrustes_angle + odom_yaw - imu_yaw))

    primary_item, mask, weights, primary_angle = primary
    rng = np.random.default_rng(20260920)
    indices = np.flatnonzero(mask)
    bootstrap = []
    for _ in range(2000):
        chosen = rng.choice(indices, len(indices), replace=True)
        angle = solve_rotation(lv[chosen], gv[chosen],
                               1.0/np.maximum(gvar[chosen].sum(axis=1), 1e-6))
        bootstrap.append(wrap_deg(math.degrees(angle + odom_yaw - imu_yaw)))
    # Unwrap bootstrap values around the primary estimate before percentiles.
    bootstrap = primary_item["plot_offset_added_to_published_wit_yaw_deg"] + np.array([
        wrap_deg(x-primary_item["plot_offset_added_to_published_wit_yaw_deg"])
        for x in bootstrap])

    report = {
        "offset_definition": "angle added to recorded /wit/imu yaw by the plotting alignment",
        "theory": {
            "ros_and_map_convention": "ENU: yaw 0=east, positive counter-clockwise",
            "urdf_wit_to_base_yaw_deg": 0.0,
            "local_driver_operation_deg": -90.0,
            "offset_that_only_cancels_local_driver_operation_deg": 90.0,
            "preferred_plot_offset_if_driver_publishes_a_correct_ros_enu_yaw_deg": 0.0,
            "limitation": "Code establishes frame conversion consistency, not whether the sensor's magnetic yaw is geographically correct.",
        },
        "common_start": start,
        "published_wit_yaw_at_start_deg": wrap_deg(math.degrees(imu_yaw)),
        "wheel_gyro_yaw_at_start_deg": wrap_deg(math.degrees(odom_yaw)),
        "initial_heading_interpretation": {
            "current_offset_90_enu_yaw_deg": wrap_deg(math.degrees(imu_yaw) + 90.0),
            "current_offset_90_bearing_clockwise_from_north_deg": (
                90.0 - (math.degrees(imu_yaw) + 90.0)) % 360.0,
            "gnss_optimal_enu_yaw_deg": wrap_deg(
                math.degrees(imu_yaw) +
                primary_item["plot_offset_added_to_published_wit_yaw_deg"]),
            "gnss_optimal_bearing_clockwise_from_north_deg": (
                90.0 - (math.degrees(imu_yaw) +
                        primary_item["plot_offset_added_to_published_wit_yaw_deg"])) % 360.0,
        },
        "gnss_velocity_primary": primary_item,
        "gnss_velocity_bootstrap_95_percent_deg": np.percentile(
            bootstrap, [2.5, 97.5]).tolist(),
        "gnss_velocity_sensitivity": velocity_solutions,
        "gnss_position_increment_sensitivity": position_solutions,
        "whole_position_procrustes": {
            "local_to_enu_rotation_deg": wrap_deg(math.degrees(procrustes_angle)),
            "plot_offset_added_to_published_wit_yaw_deg": procrustes_offset,
            "warning": "Strongly sensitive to GNSS drift and correlated samples",
        },
        "gnss_quality_warning": {
            "rtk_fix_count": 0,
            "horizontal_position_sigma_combined_m_median": summary[
                "gnss_horizontal_sigma_m_median"],
        },
    }

    offsets = np.linspace(-180, 180, 721)
    velocity_cost = []
    position_cost = []
    pos_source, pos_target = increment_sets.get(20, next(iter(increment_sets.values())))
    for offset in offsets:
        angle = imu_yaw + math.radians(offset) - odom_yaw
        vpred = lv[mask] @ rotation(angle).T
        velocity_cost.append(np.sqrt(np.average(
            np.sum((vpred-gv[mask])**2, axis=1), weights=weights)))
        ppred = pos_source @ rotation(angle).T
        position_cost.append(np.sqrt(np.mean(np.sum((ppred-pos_target)**2, axis=1))))

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    axes[0].plot(offsets, velocity_cost, label="GNSS velocity vector RMSE")
    axes[0].plot(offsets, np.array(position_cost)/max(position_cost)*max(velocity_cost),
                 label="20 s position-increment cost (scaled)")
    axes[0].axvline(0, color="gray", ls=":", label="0°")
    axes[0].axvline(90, color="tab:red", ls="--", label="current +90°")
    axes[0].axvline(primary_item["plot_offset_added_to_published_wit_yaw_deg"],
                    color="tab:green", ls="--", label="GNSS-velocity optimum")
    axes[0].set(xlabel="Offset added to published Wit yaw [deg]",
                ylabel="Objective", title="Offset objective")

    residual = angle_residual(lv[mask], gv[mask], primary_angle)
    axes[1].plot(vt[mask]-start, residual, "o-", ms=3,
                 label="course residual")
    axes[1].axhline(0, color="black", lw=.8)
    axes[1].set(xlabel="Time since common start [s]",
                ylabel="GNSS course - rotated odom course [deg]",
                title="Primary GNSS-velocity residual")

    local_centered = lxy-lxy[0]
    gnss_centered = gxy-gxy[0]
    axes[2].plot(gnss_centered[:, 0], gnss_centered[:, 1], "k.-", label="GNSS position")
    for offset, style in [(0, ":"), (90, "--"),
                          (primary_item["plot_offset_added_to_published_wit_yaw_deg"], "-")]:
        angle = imu_yaw + math.radians(offset) - odom_yaw
        path = local_centered @ rotation(angle).T
        axes[2].plot(path[:, 0], path[:, 1], style, label=f"wheel_gyro offset {offset:.1f}°")
    axes[2].set(xlabel="East [m]", ylabel="North [m]",
                title="Position cross-check", aspect="equal")
    for ax in axes:
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "heading_offset_analysis.png", dpi=160)
    (out / "heading_offset_analysis.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
