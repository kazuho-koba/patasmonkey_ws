#!/usr/bin/env python3
"""Replay recorded EKF inputs into selected Foxy robot_localization instances.

Only recorded wheel/VIO odometry, Wit IMU and static TF are republished.
No hardware, dynamic TF, existing EKF output or GNSS is fed to the estimators.
MCAP is read directly, so the rosbag2 MCAP storage plugin is not required.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import numpy as np
import yaml
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages

from pm_evaluation.cli.plot_bag_trajectories import (
    find_mcap_files, infer_rtk_state_from_msg, quaternion_to_yaw,
)

SOURCES = ("wheel", "wheel_imu", "wheel_gyro", "wheel_gyro_vio", "vio")
ODOMS = {"/wheel/odometry": "raw_wheel", "/vio/odometry": "raw_vio",
         "/odometry/local": "recorded_local"}
INPUTS = ("/wheel/odometry", "/wit/imu", "/vio/odometry", "/tf_static")
SOURCE_INPUTS = {
    "wheel": {"/wheel/odometry"},
    "wheel_imu": {"/wheel/odometry", "/wit/imu"},
    "wheel_gyro": {"/wheel/odometry", "/wit/imu"},
    "wheel_gyro_vio": {"/wheel/odometry", "/wit/imu", "/vio/odometry"},
    "vio": {"/vio/odometry"},
}
COLUMNS = "t,x,y,z,yaw,vx,vy,vz,wz,var_vx,var_vy,var_wz".split(",")


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw(q):
    return quaternion_to_yaw(q.x, q.y, q.z, q.w)


def odom_row(msg):
    p, v = msg.pose.pose, msg.twist.twist
    c = msg.twist.covariance
    return [stamp(msg), p.position.x, p.position.y, p.position.z,
            yaw(p.orientation), v.linear.x, v.linear.y, v.linear.z,
            v.angular.z, c[0], c[7], c[35]]


def save_csv(path, rows, columns):
    with path.open("w") as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        writer.writerows(rows)


def extract(bag, out, sources):
    odom_topics = set()
    if any(k.startswith("wheel") for k in sources):
        odom_topics.add("/wheel/odometry")
    if any("vio" in k for k in sources):
        odom_topics.add("/vio/odometry")
    rows = {ODOMS[k]: [] for k in odom_topics}
    imu, fixes, fix_velocities, rtk = [], [], [], []
    audit = {}
    for path in find_mcap_files(bag):
        print("Extracting", path.name, flush=True)
        for item in read_ros2_messages(path, topics=list(odom_topics) +
                                      ["/wit/imu", "/fix", "/fix_velocity", "/navpvt"]):
            topic, msg = item.channel.topic, item.ros_msg
            t = stamp(msg) if hasattr(msg, "header") else item.log_time_ns * 1e-9
            a = audit.setdefault(topic, {"count": 0, "frames": [], "delays": []})
            a["count"] += 1
            a["delays"].append(item.log_time_ns * 1e-9 - t)
            frame = getattr(getattr(msg, "header", None), "frame_id", "")
            if hasattr(msg, "child_frame_id"):
                frame += " -> " + msg.child_frame_id
            if frame not in a["frames"]:
                a["frames"].append(frame)
            if topic in odom_topics:
                rows[ODOMS[topic]].append(odom_row(msg))
            elif topic == "/wit/imu":
                imu.append([t, yaw(msg.orientation), msg.angular_velocity.z,
                            msg.orientation_covariance[8], msg.angular_velocity_covariance[8]])
            elif topic == "/fix":
                fixes.append([t, msg.longitude, msg.latitude, msg.status.status,
                              msg.position_covariance[0], msg.position_covariance[4]])
            elif topic == "/fix_velocity":
                velocity = msg.twist.twist.linear
                covariance = msg.twist.covariance
                fix_velocities.append([t, velocity.x, velocity.y, velocity.z,
                                       covariance[0], covariance[7]])
            else:
                rtk.append([t, infer_rtk_state_from_msg(msg)])
    for k, values in rows.items():
        if not values:
            raise RuntimeError("Missing required recorded topic: " + k)
        save_csv(out / (k + ".csv"), values, COLUMNS)
    save_csv(out / "imu.csv", imu, ["t", "yaw", "wz", "var_yaw", "var_wz"])
    save_csv(out / "fix.csv", fixes, ["t", "lon", "lat", "status", "var_e", "var_n"])
    save_csv(out / "fix_velocity.csv", fix_velocities,
             ["t", "east_mps", "north_mps", "up_mps", "var_e", "var_n"])
    save_csv(out / "rtk.csv", rtk, ["t", "state"])
    for a in audit.values():
        delays = a.pop("delays")
        a["record_minus_header_sec_p0_p50_p100"] = np.percentile(delays, [0, 50, 100]).tolist()
    (out / "input_audit.json").write_text(json.dumps(audit, indent=2))


def replay(bag, out, config, rate, sources):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    selected_inputs = set().union(*(SOURCE_INPUTS[k] for k in sources)) | {"/tf_static"}
    events, types, static = [], {}, {}
    for path in find_mcap_files(bag):
        with path.open("rb") as f:
            for schema, channel, message in make_reader(f).iter_messages(topics=selected_inputs):
                topic = channel.topic
                types[topic] = get_message(schema.name)
                if topic == "/tf_static":
                    msg = deserialize_message(message.data, types[topic])
                    for tf in msg.transforms:
                        static[tf.child_frame_id] = tf
                else:
                    events.append((message.log_time, topic, message.data))
    events.sort(key=lambda e: e[0])
    if not events or not static or any(t not in types for t in selected_inputs):
        raise RuntimeError("Bag is missing a selected input or static TF")
    rclpy.init()
    node = rclpy.create_node("source_comparison_replay")
    if node.get_node_names().count("source_comparison_replay") > 1:
        raise RuntimeError("ROS domain already in use by another comparison")
    pubs = {t: node.create_publisher(types[t], t, 1000)
            for t in selected_inputs if t != "/tf_static"}
    clock = node.create_publisher(Clock, "/clock", 100)
    tf_pub = node.create_publisher(types["/tf_static"], "/tf_static",
                                   QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    tf_msg = types["/tf_static"]()
    tf_msg.transforms = list(static.values())
    results = {k: [] for k in sources}
    subs = [node.create_subscription(Odometry, "/evaluation/" + k,
                                    lambda msg, key=k: results[key].append(odom_row(msg)), 1000)
            for k in sources]
    processes, logs = [], []
    executable = str(Path(get_package_prefix("robot_localization")) / "lib/robot_localization/ekf_node")
    try:
        for k in sources:
            log = (out / (k + ".log")).open("w")
            logs.append(log)
            cfg = config / ("source_" + k + ".yaml")
            shutil.copy2(cfg, out / cfg.name)
            processes.append(subprocess.Popen([
                executable, "--ros-args", "--params-file", str(cfg),
                "-r", "__node:=evaluation_" + k,
                "-r", "odometry/filtered:=/evaluation/" + k], stdout=log, stderr=log))
        deadline = time.monotonic() + 20
        expected = {topic: sum(topic in SOURCE_INPUTS[k] for k in sources)
                    for topic in selected_inputs if topic != "/tf_static"}
        while any(pubs[t].get_subscription_count() < n for t, n in expected.items()):
            if time.monotonic() > deadline or any(p.poll() is not None for p in processes):
                raise RuntimeError("EKF startup failed; inspect output logs")
            rclpy.spin_once(node, timeout_sec=0.05)
        tf_pub.publish(tf_msg)
        first, last = events[0][0], events[-1][0]
        msg_clock = Clock()

        def set_clock(ns):
            msg_clock.clock.sec = ns // 1000000000
            msg_clock.clock.nanosec = ns % 1000000000
            clock.publish(msg_clock)

        set_clock(first)
        start_wait = time.monotonic() + 1
        while time.monotonic() < start_wait:
            rclpy.spin_once(node, timeout_sec=0.01)
        wall_start, index, next_report = time.monotonic(), 0, 0
        # 100 Hz simulation clock; original receive ordering and header stamps retained.
        for now in range(first, last + 10000001, 10000000):
            due = wall_start + (now - first) * 1e-9 / rate
            while time.monotonic() < due:
                rclpy.spin_once(node, timeout_sec=max(0.0, min(0.002, due - time.monotonic())))
            set_clock(now)
            while index < len(events) and events[index][0] <= now:
                _, topic, data = events[index]
                pubs[topic].publish(deserialize_message(data, types[topic]))
                index += 1
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0)
            elapsed = (now - first) * 1e-9
            if elapsed >= next_report:
                print("Replay %.1f s / %.1f s; output counts %s" %
                      (elapsed, (last - first) * 1e-9, {k: len(v) for k, v in results.items()}), flush=True)
                next_report += 30
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("An EKF exited during playback")
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        for k, rows in results.items():
            if len(rows) < 2:
                raise RuntimeError("No usable output: " + k)
            save_csv(out / (k + ".csv"), rows, COLUMNS)
    finally:
        for p in processes:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


def load(out, name):
    a = np.loadtxt(out / (name + ".csv"), delimiter=",", skiprows=1, ndmin=2)
    a = a[np.isfinite(a).all(axis=1)]
    a = a[np.argsort(a[:, 0], kind="stable")]
    return a[np.r_[True, np.diff(a[:, 0]) > 0]]


def rotate(points, angle):
    c, s = np.cos(angle), np.sin(angle)
    return points @ np.array([[c, s], [-s, c]])


def plot(out, heading_offset, basemap, sources, suffix=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from pyproj import CRS, Transformer

    support = []
    if any(k.startswith("wheel") for k in sources):
        support.append("raw_wheel")
    if any("vio" in k for k in sources):
        support.append("raw_vio")
    data = {k: load(out, k) for k in tuple(sources) + tuple(support)}
    imu, fix = load(out, "imu"), load(out, "fix")
    fix = fix[(fix[:, 3] >= 0) & (abs(fix[:, 2]) < 90) & (abs(fix[:, 1]) <= 180)]
    if len(fix) < 2:
        raise RuntimeError("Insufficient valid GNSS fixes")
    start = max(a[0, 0] for a in list(data.values()) + [imu, fix]) + 1
    end = min(a[-1, 0] for a in list(data.values()) + [imu, fix])
    if end <= start:
        raise RuntimeError("No common comparison interval")
    lon, lat = fix[0, 1:3]
    local = CRS.from_proj4("+proj=aeqd +lat_0=%s +lon_0=%s +datum=WGS84 +units=m" % (lat, lon))
    to_local = Transformer.from_crs(4326, local, always_xy=True)
    to_map = Transformer.from_crs(local, 3857, always_xy=True)
    gx, gy = to_local.transform(fix[:, 1], fix[:, 2])
    g = np.column_stack([gx, gy])
    origin = np.array([np.interp(start, fix[:, 0], g[:, j]) for j in range(2)])
    heading = np.interp(start, imu[:, 0], np.unwrap(imu[:, 1])) + math.radians(heading_offset)
    aligned, stats = {}, {}
    for k, a in data.items():
        xy0 = np.array([np.interp(start, a[:, 0], a[:, j]) for j in [1, 2]])
        yaw0 = np.interp(start, a[:, 0], np.unwrap(a[:, 4]))
        use = (a[:, 0] >= start) & (a[:, 0] <= end)
        xy = rotate(a[use, 1:3] - xy0, heading - yaw0) + origin
        aligned[k] = (a[use, 0], xy)
        gt_mask = (fix[:, 0] >= a[use, 0][0]) & (fix[:, 0] <= a[use, 0][-1])
        gt_t = fix[gt_mask, 0]
        interp = np.column_stack([np.interp(gt_t, a[use, 0], xy[:, j]) for j in range(2)])
        errors = np.linalg.norm(interp - g[gt_mask], axis=1)
        stats[k] = {"samples_full": len(a), "start_header": float(a[0, 0]),
                    "end_header": float(a[-1, 0]), "max_header_gap_s": float(np.diff(a[:, 0]).max()),
                    "initial_map_rotation_deg": float(np.degrees(heading - yaw0)),
                    "distance_common_m": float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
                    "closure_error_common_m": float(np.linalg.norm(xy[-1] - xy[0])),
                    "gnss_disagreement_rmse_m": float(np.sqrt(np.mean(errors ** 2))),
                    "gnss_disagreement_final_m": float(errors[-1])}
        save_csv(out / (k + "_aligned" + suffix + ".csv"),
                 np.column_stack([a[use, 0], xy]), ["t", "east_m", "north_m"])
    gm = np.column_stack(to_map.transform(g[:, 0], g[:, 1]))
    mapped = {k: np.column_stack(to_map.transform(xy[:, 0], xy[:, 1])) for k, (_, xy) in aligned.items()}
    gmask = (fix[:, 0] >= start) & (fix[:, 0] <= end)
    center = gm[gmask].mean(axis=0)
    half = max(np.ptp(gm[gmask], axis=0).max() * 0.65, 30)
    bounds = (center[0]-half, center[0]+half, center[1]-half, center[1]+half)
    image, extent, map_error = None, None, None
    if basemap:
        try:
            import contextily as ctx
            ctx.tile.USER_AGENT = "PatasmonkeySourceComparison/1.0"
            # One bounded OSM request, shared by every plot.
            cache = out / "basemap.npz"
            if cache.exists():
                cached = np.load(cache)
                image, extent = cached["image"], cached["extent"]
            else:
                image, extent = ctx.bounds2img(bounds[0], bounds[2], bounds[1], bounds[3],
                                              zoom=18, source=ctx.providers.OpenStreetMap.Mapnik)
                np.savez_compressed(cache, image=image, extent=extent)
        except Exception as exc:
            map_error = str(exc)
            print("OSM unavailable:", exc, flush=True)
    colors = {"wheel": "tab:blue", "wheel_imu": "tab:orange",
              "wheel_gyro": "tab:purple", "wheel_gyro_vio": "tab:pink",
              "vio": "tab:green",
              "raw_wheel": "tab:cyan", "raw_vio": "tab:olive", "recorded_local": "tab:red"}

    def decorate(ax, full=False):
        if image is not None:
            ax.imshow(image, extent=extent, interpolation="bilinear", zorder=0)
            ax.text(.01, .01, "© OpenStreetMap contributors", transform=ax.transAxes, fontsize=7)
        ax.plot(gm[gmask, 0], gm[gmask, 1], "k.", ms=3, label="GNSS (not ground truth)")
        if not full:
            ax.set_xlim(bounds[:2])
            ax.set_ylim(bounds[2:])
        ax.set_aspect("equal")
        ax.set_xlabel("Web Mercator East [projected m]")
        ax.set_ylabel("Web Mercator North [projected m]")
        ax.ticklabel_format(useOffset=True, style="plain")
        ax.grid(alpha=.2)

    for full in [False, True]:
        fig, ax = plt.subplots(figsize=(10, 9))
        decorate(ax, full)
        for k in sources:
            xy = mapped[k]
            ax.plot(xy[:, 0], xy[:, 1], color=colors[k], label=k, lw=1.6)
        if full:
            all_xy = np.vstack([gm[gmask]] + [mapped[k] for k in sources])
            lo, hi = all_xy.min(axis=0), all_xy.max(axis=0)
            margin = max((hi-lo).max() * .05, 5)
            ax.set_xlim(lo[0]-margin, hi[0]+margin)
            ax.set_ylim(lo[1]-margin, hi[1]+margin)
        ax.legend()
        ax.set_title("Source-isolated EKF%s | shared time and initial pose\nIMU heading offset: %+g deg; no scale/whole-path fitting" %
                     ("s" if len(sources) != 1 else "", heading_offset))
        fig.tight_layout()
        name = "comparison_full" if full else "comparison_map"
        fig.savefig(out / (name + suffix + ".png"), dpi=160)
        plt.close(fig)
    fig, axes_grid = plt.subplots(2, 3, figsize=(19, 12))
    axes = axes_grid.ravel()
    panel_points = np.vstack([gm[gmask]] + list(mapped.values()))
    panel_low, panel_high = panel_points.min(axis=0)-5, panel_points.max(axis=0)+5
    for ax, k in zip(axes, sources):
        decorate(ax)
        ax.set_xlim(panel_low[0], panel_high[0])
        ax.set_ylim(panel_low[1], panel_high[1])
        for name in [k] + {"wheel": ["raw_wheel"], "wheel_imu": [],
                           "wheel_gyro": [], "wheel_gyro_vio": [],
                           "vio": ["raw_vio"]}[k]:
            xy = mapped[name]
            ax.plot(xy[:, 0], xy[:, 1], color=colors[name], label=name)
        ax.set_title(k)
        ax.legend(fontsize=8)
    for ax in axes[len(sources):]:
        ax.set_visible(False)
    fig.tight_layout()
    fig.savefig(out / ("source_panels" + suffix + ".png"), dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    for k in support:
        a = data[k]
        axes[0].plot(a[:, 0]-start, a[:, 5], label=k, lw=.7)
        axes[1].plot(a[:, 0]-start, a[:, 8], label=k, lw=.7)
        axes[2].plot(a[:, 0]-start, np.degrees(np.unwrap(a[:, 4])-np.interp(start, a[:, 0], np.unwrap(a[:, 4]))), label=k, lw=.7)
    axes[1].plot(imu[:, 0]-start, imu[:, 2], label="wit gyro z", lw=.7)
    axes[2].plot(imu[:, 0]-start, np.degrees(np.unwrap(imu[:, 1])-np.interp(start, imu[:, 0], np.unwrap(imu[:, 1]))), label="wit yaw", lw=.7)
    for ax, label in zip(axes, ["Body vx [m/s]", "Body wz [rad/s]", "Relative yaw [deg]"]):
        ax.set_ylabel(label)
        ax.legend()
        ax.grid(alpha=.3)
        ax.set_xlim(0, end-start)
    axes[-1].set_xlabel("Time since common anchor [s]")
    fig.tight_layout()
    fig.savefig(out / ("input_signals" + suffix + ".png"), dpi=160)
    plt.close(fig)
    rtk_counts = {}
    with (out / "rtk.csv").open() as f:
        for row in csv.DictReader(f):
            rtk_counts[row["state"]] = rtk_counts.get(row["state"], 0) + 1
    report = {"common_start": start, "common_end": end, "common_duration_s": end-start,
              "heading_offset_deg": heading_offset, "alignment": "shared time, GNSS translation, IMU yaw + offset minus each pose yaw; no scale fit",
              "gnss_horizontal_sigma_m_median": float(np.median(np.sqrt(fix[:, 4]+fix[:, 5]))),
              "gnss_navpvt_states": rtk_counts, "basemap_available": image is not None,
              "basemap_error": map_error, "metrics": stats}
    grid = np.arange(start, end, .02)
    wheel = data.get("raw_wheel")
    vio = data.get("raw_vio")
    iz = np.interp(grid, imu[:, 0], imu[:, 2])
    report["input_comparisons"] = {}
    comparisons = []
    if wheel is not None:
        wv = np.interp(grid, wheel[:, 0], wheel[:, 5])
        wz = np.interp(grid, wheel[:, 0], wheel[:, 8])
        comparisons.append(("wheel_wz_vs_imu_wz", wz, iz))
    if vio is not None:
        vv = np.interp(grid, vio[:, 0], vio[:, 5])
        vz = np.interp(grid, vio[:, 0], vio[:, 8])
        comparisons.append(("vio_wz_vs_imu_wz", vz, iz))
        if wheel is not None:
            comparisons.append(("wheel_vx_vs_vio_vx", wv, vv))
    for name, a, b in comparisons:
        report["input_comparisons"][name] = {
            "correlation": float(np.corrcoef(a, b)[0, 1]),
            "rmse": float(np.sqrt(np.mean((a-b)**2))),
            "zero_intercept_slope_first_over_second": float(np.dot(a, b)/np.dot(b, b))}
    # Wheel velocity is ds/dt over the PRECEDING encoder interval. Integrating
    # interpolated irregular samples instead introduces a large artificial bias.
    report["integrated_wz_common_deg"] = {}
    if wheel is not None:
        dt = np.maximum(0, np.minimum(wheel[1:, 0], end) -
                        np.maximum(wheel[:-1, 0], start))
        report["integrated_wz_common_deg"]["wheel_preceding_interval"] = float(
            np.degrees(np.dot(dt, wheel[1:, 8])))
    angular_sources = [("imu_trapezoid", imu, 2)]
    if vio is not None:
        angular_sources.append(("vio_trapezoid", vio, 8))
    for k, a, col in angular_sources:
        times = np.r_[start, a[(a[:, 0] > start) & (a[:, 0] < end), 0], end]
        report["integrated_wz_common_deg"][k] = float(np.degrees(
            np.trapz(np.interp(times, a[:, 0], a[:, col]), times)))
    report["yaw_disagreement_vs_imu_deg_p5_p50_p95"] = {}
    imu_yaw = np.interp(grid, imu[:, 0], np.unwrap(imu[:, 1]))
    for k, a in data.items():
        angles = np.interp(grid, a[:, 0], np.unwrap(a[:, 4]))
        diff = (angles - angles[0]) - (imu_yaw - imu_yaw[0])
        report["yaw_disagreement_vs_imu_deg_p5_p50_p95"][k] = np.percentile(
            np.degrees(diff), [5, 50, 95]).tolist()
    report["pose_yaw_change_common_deg"] = {
        k: float(np.degrees(np.interp(end, a[:, 0], np.unwrap(a[:, 4])) -
                            np.interp(start, a[:, 0], np.unwrap(a[:, 4]))))
        for k, a in data.items()}
    report["pose_yaw_change_common_deg"]["imu"] = float(np.degrees(
        np.interp(end, imu[:, 0], np.unwrap(imu[:, 1])) -
        np.interp(start, imu[:, 0], np.unwrap(imu[:, 1]))))
    (out / ("summary" + suffix + ".json")).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--domain-id", type=int, default=87)
    parser.add_argument("--heading-offset-deg", type=float, default=90.0,
                        help="Display convention inherited from plot_bag_trajectories, not calibration")
    parser.add_argument("--plot-only", action="store_true")
    parser.add_argument("--no-basemap", action="store_true")
    parser.add_argument("--output-suffix", default="",
                        help="Suffix for plot, aligned CSV, and summary filenames")
    parser.add_argument("--sources", nargs="+", choices=SOURCES, default=list(SOURCES),
                        help="EKF conditions to run and plot (default: all)")
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error("rate must be positive")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if not args.plot_only:
        if any((args.output / (k + ".csv")).exists() for k in args.sources):
            parser.error("Output already contains a replay; use a new directory or --plot-only")
        os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        from ament_index_python.packages import get_package_share_directory
        config = Path(get_package_share_directory("pm_evaluation")) / "config"
        extract(args.bag, args.output, args.sources)
        replay(args.bag, args.output, config, args.rate, args.sources)
        (args.output / "run.json").write_text(json.dumps({
            "bag": str(args.bag.resolve()), "rate": args.rate, "domain": args.domain_id,
            "input_time": "original bag receive order; original header stamps; 100Hz /clock",
            "configs": ["source_" + k + ".yaml" for k in args.sources]}, indent=2))
    suffix = args.output_suffix
    if suffix and not suffix.startswith("_"):
        suffix = "_" + suffix
    plot(args.output, args.heading_offset_deg, not args.no_basemap, args.sources, suffix)


if __name__ == "__main__":
    main()
