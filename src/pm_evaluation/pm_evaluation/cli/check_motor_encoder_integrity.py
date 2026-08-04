#!/usr/bin/env python3
"""
/motor_state の pos_turns 健全性をコンソール診断する。

確認項目:
  1. 位置飛び・固着・方向不一致・リセット疑い
  2. reported RPS積分値とpos_turns差分の残差
  3. MADによる残差外れ値
  4. 残差のオドメトリ距離・yaw影響換算
  5. 停止区間の位置ドリフト
  6. stamp、MCAP log_time、メッセージ周期の品質

注意:
  MotorStateには左右position/velocityの個別読取時刻がないため、
  ODriveへの各USB問い合わせ間の時間差はbagだけでは判定できない。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
from mcap_ros2.reader import read_ros2_messages


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Check MotorState encoder integrity")
    p.add_argument("bag", type=Path)
    p.add_argument("--topic", default="/motor_state")
    p.add_argument("--wheel-radius", type=float, default=0.1016)
    p.add_argument("--tread-width", type=float, default=0.36)
    p.add_argument("--gear-ratio", type=float, default=10.0)
    p.add_argument("--max-motor-rps", type=float, default=40.0)
    p.add_argument("--speed-margin", type=float, default=1.25)
    p.add_argument("--moving-rps", type=float, default=2.0)
    p.add_argument("--direction-min-rps", type=float, default=1.0)
    p.add_argument("--position-epsilon-turns", type=float, default=1e-6)
    p.add_argument("--stale-consecutive", type=int, default=3)
    p.add_argument("--reset-near-zero-turns", type=float, default=1.0)
    p.add_argument("--mad-sigma", type=float, default=6.0)
    p.add_argument("--mad-sigma-floor-turns", type=float, default=1e-4)
    p.add_argument("--odom-distance-threshold-m", type=float, default=0.002)
    p.add_argument("--odom-yaw-threshold-deg", type=float, default=0.5)
    p.add_argument("--stop-cmd-rps", type=float, default=0.05)
    p.add_argument("--stop-reported-rps", type=float, default=0.2)
    p.add_argument("--stop-min-duration-sec", type=float, default=1.0)
    p.add_argument("--timing-gap-factor", type=float, default=3.0)
    p.add_argument("--top-events", type=int, default=10)
    return p.parse_args()


def validate_args(a: argparse.Namespace) -> None:
    positive = (
        "wheel_radius", "tread_width", "gear_ratio", "max_motor_rps",
        "speed_margin", "moving_rps", "direction_min_rps",
        "position_epsilon_turns", "stale_consecutive",
        "reset_near_zero_turns", "mad_sigma", "mad_sigma_floor_turns",
        "odom_distance_threshold_m", "odom_yaw_threshold_deg",
        "stop_min_duration_sec", "timing_gap_factor", "top_events",
    )
    for name in positive:
        if getattr(a, name) <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be positive")
    if a.stop_cmd_rps < 0 or a.stop_reported_rps < 0:
        raise ValueError("Stop thresholds must be nonnegative")


def sort_key(path: Path) -> Tuple[str, int]:
    prefix, sep, suffix = path.stem.rpartition("_")
    return (prefix, int(suffix)) if sep and suffix.isdigit() else (path.stem, -1)


def find_mcap_files(path: Path) -> List[Path]:
    path = path.expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(path)
    if path.is_file():
        if path.suffix.lower() != ".mcap":
            raise ValueError(f"Not an MCAP file: {path}")
        return [path]
    files = sorted(path.glob("*.mcap"), key=sort_key)
    if not files:
        raise ValueError(f"No MCAP files found in {path}")
    return files


def msg_stamp_ns(msg) -> Tuple[int, bool]:
    try:
        sec = int(msg.stamp.sec)
        nsec = int(msg.stamp.nanosec)
        return sec * 1_000_000_000 + nsec, (sec != 0 or nsec != 0)
    except Exception:
        return 0, False


def read_data(files: Sequence[Path], topic: str) -> dict:
    rows = []
    required = (
        "left_cmd_rps", "right_cmd_rps", "left_pos_turns",
        "right_pos_turns", "left_vel_rps", "right_vel_rps",
    )
    for path in files:
        print(f"[READ] {path}")
        for decoded in read_ros2_messages(path, topics={topic}, log_time_order=True):
            if decoded.channel.topic != topic:
                continue
            msg = decoded.ros_msg
            missing = [name for name in required if not hasattr(msg, name)]
            if missing:
                raise AttributeError(f"{topic} missing fields: {missing}")
            log_ns = int(decoded.log_time_ns)
            stamp_ns, stamp_valid = msg_stamp_ns(msg)
            effective_ns = stamp_ns if stamp_valid else log_ns
            rows.append((
                log_ns, stamp_ns, effective_ns, stamp_valid,
                float(msg.left_cmd_rps), float(msg.right_cmd_rps),
                float(msg.left_pos_turns), float(msg.right_pos_turns),
                float(msg.left_vel_rps), float(msg.right_vel_rps),
            ))
    if not rows:
        raise RuntimeError(f"No messages found on {topic}")

    # wheel_odometry_nodeが受信する順序に近いlog_time順を維持する。
    rows.sort(key=lambda row: row[0])
    c = list(zip(*rows))
    d = {
        "log_ns": np.asarray(c[0], dtype=np.int64),
        "stamp_ns": np.asarray(c[1], dtype=np.int64),
        "time_ns": np.asarray(c[2], dtype=np.int64),
        "stamp_valid": np.asarray(c[3], dtype=bool),
        "lcmd": np.asarray(c[4], dtype=float),
        "rcmd": np.asarray(c[5], dtype=float),
        "lpos": np.asarray(c[6], dtype=float),
        "rpos": np.asarray(c[7], dtype=float),
        "lvel": np.asarray(c[8], dtype=float),
        "rvel": np.asarray(c[9], dtype=float),
    }
    d["t"] = (d["time_ns"] - d["time_ns"][0]).astype(float) * 1e-9
    return d


def nan_diff(values: np.ndarray) -> np.ndarray:
    out = np.full(len(values), np.nan)
    if len(values) > 1:
        out[1:] = np.diff(values)
    return out


def consecutive_mask(mask: np.ndarray, minimum: int) -> np.ndarray:
    out = np.zeros(len(mask), dtype=bool)
    start: Optional[int] = None
    for i, value in enumerate(mask):
        if value and start is None:
            start = i
        if (not value or i == len(mask) - 1) and start is not None:
            end = i if value and i == len(mask) - 1 else i - 1
            if end - start + 1 >= minimum:
                out[start:end + 1] = True
            start = None
    return out


def side_diagnostics(pos: np.ndarray, vel: np.ndarray, dt: np.ndarray,
                     timing_valid: np.ndarray, a: argparse.Namespace) -> dict:
    dp = nan_diff(pos)
    pos_rps = np.full(len(pos), np.nan)
    valid = timing_valid & np.isfinite(dp) & np.isfinite(pos)
    pos_rps[valid] = dp[valid] / dt[valid]

    vel_mid = np.full(len(vel), np.nan)
    if len(vel) > 1:
        vel_mid[1:] = 0.5 * (vel[:-1] + vel[1:])

    expected_dp = vel_mid * dt
    residual = dp - expected_dp
    nonfinite = ~np.isfinite(pos)
    jump = np.isfinite(pos_rps) & (
        np.abs(pos_rps) > a.max_motor_rps * a.speed_margin
    )
    stale_candidate = (
        timing_valid & np.isfinite(dp) & np.isfinite(vel_mid)
        & (np.abs(dp) <= a.position_epsilon_turns)
        & (np.abs(vel_mid) >= a.moving_rps)
    )
    stale = consecutive_mask(stale_candidate, a.stale_consecutive)
    direction = (
        timing_valid & np.isfinite(pos_rps) & np.isfinite(vel_mid)
        & (np.abs(pos_rps) >= a.direction_min_rps)
        & (np.abs(vel_mid) >= a.direction_min_rps)
        & (pos_rps * vel_mid < 0)
    )
    reset = jump & np.isfinite(pos) & (np.abs(pos) <= a.reset_near_zero_turns)

    mad_valid = timing_valid & ~jump & ~nonfinite & np.isfinite(residual)
    mad_mask = np.zeros(len(pos), dtype=bool)
    if np.any(mad_valid):
        values = residual[mad_valid]
        med = float(np.median(values))
        mad = float(np.median(np.abs(values - med)))
        robust_sigma = max(1.4826 * mad, a.mad_sigma_floor_turns)
        mad_mask[mad_valid] = np.abs(residual[mad_valid] - med) > a.mad_sigma * robust_sigma
    else:
        med = mad = robust_sigma = math.nan

    return {
        "dp": dp, "pos_rps": pos_rps, "vel_mid": vel_mid,
        "expected_dp": expected_dp, "residual": residual,
        "nonfinite": nonfinite, "jump": jump,
        "stale_candidate": stale_candidate, "stale": stale,
        "direction": direction, "reset": reset, "mad": mad_mask,
        "residual_median": med, "residual_mad": mad,
        "robust_sigma": robust_sigma,
    }


def finite_metric(values: np.ndarray, mode: str) -> float:
    x = values[np.isfinite(values)]
    if x.size == 0:
        return math.nan
    if mode == "mae":
        return float(np.mean(np.abs(x)))
    if mode == "rmse":
        return float(np.sqrt(np.mean(x * x)))
    if mode == "max":
        return float(np.max(np.abs(x)))
    raise ValueError(mode)


def f(value: float, digits: int = 6) -> str:
    return "unavailable" if not math.isfinite(value) else f"{value:.{digits}f}"


def heading(text: str) -> None:
    print("\n" + "=" * 88)
    print(text)
    print("=" * 88)


def count(label: str, mask: np.ndarray) -> None:
    print(f"  {label:<55}: {np.count_nonzero(mask):8d}")


def print_side(name: str, s: dict) -> None:
    print(f"\n  [{name}]")
    count("Non-finite cumulative position", s["nonfinite"])
    count("Physical-limit position jump", s["jump"])
    count("Moving but unchanged-position candidates", s["stale_candidate"])
    count("Consecutive stale-position samples", s["stale"])
    count("Direction mismatch", s["direction"])
    count("Position reset suspected", s["reset"])
    count("MAD residual anomalies", s["mad"])
    print(f"  {'Residual MAE':<55}: {f(finite_metric(s['residual'], 'mae'))} turns")
    print(f"  {'Residual RMSE':<55}: {f(finite_metric(s['residual'], 'rmse'))} turns")
    print(f"  {'Residual maximum absolute':<55}: {f(finite_metric(s['residual'], 'max'))} turns")
    print(f"  {'Residual median':<55}: {f(s['residual_median'])} turns")
    print(f"  {'Residual MAD':<55}: {f(s['residual_mad'])} turns")
    print(f"  {'Residual robust sigma':<55}: {f(s['robust_sigma'])} turns")


def print_events(title: str, mask: np.ndarray, d: dict, l: dict, r: dict,
                 dt: np.ndarray, limit: int) -> None:
    idx = np.flatnonzero(mask)
    print(f"\n  {title}: {len(idx)} sample(s)")
    if len(idx) == 0:
        return
    print("    time[s]    dt[s]   L_posRPS   R_posRPS   L_velMid   R_velMid   L_dpos      R_dpos")
    for i in idx[:limit]:
        print(
            f"    {d['t'][i]:8.3f} {dt[i]:8.4f} "
            f"{l['pos_rps'][i]:10.3f} {r['pos_rps'][i]:10.3f} "
            f"{l['vel_mid'][i]:10.3f} {r['vel_mid'][i]:10.3f} "
            f"{l['dp'][i]:10.6f} {r['dp'][i]:10.6f}"
        )
    if len(idx) > limit:
        print(f"    ... {len(idx) - limit} additional sample(s) omitted")


def true_segments(mask: np.ndarray) -> List[Tuple[int, int]]:
    result = []
    start: Optional[int] = None
    for i, value in enumerate(mask):
        if value and start is None:
            start = i
        if (not value or i == len(mask) - 1) and start is not None:
            end = i if value and i == len(mask) - 1 else i - 1
            result.append((start, end))
            start = None
    return result


def timestamp_check(d: dict, a: argparse.Namespace) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    dt = nan_diff(d["time_ns"].astype(float) * 1e-9)
    nonpositive = np.isfinite(dt) & (dt <= 0)
    positive = dt[np.isfinite(dt) & (dt > 0)]
    median_dt = float(np.median(positive)) if positive.size else math.nan
    gap = np.zeros(len(dt), dtype=bool)
    if math.isfinite(median_dt):
        gap = np.isfinite(dt) & (dt > a.timing_gap_factor * median_dt)
    valid = np.isfinite(dt) & (dt > 0)

    heading("6. Timestamp and message timing diagnostics")
    print(f"Samples                                             : {len(dt)}")
    print(f"MotorState.stamp missing/fallback samples           : {np.count_nonzero(~d['stamp_valid'])}")
    print(f"Non-positive effective timestamp intervals          : {np.count_nonzero(nonpositive)}")
    print(f"Timing gaps > {a.timing_gap_factor:g} x median dt                   : {np.count_nonzero(gap)}")
    if positive.size:
        print(f"Median effective dt                                 : {np.median(positive):.6f} s")
        print(f"Estimated message rate                              : {1 / np.median(positive):.3f} Hz")
        print(f"95th percentile effective dt                        : {np.percentile(positive, 95):.6f} s")
        print(f"Maximum effective dt                                : {np.max(positive):.6f} s")

    if np.any(d["stamp_valid"]):
        v = d["stamp_valid"]
        latency_ms = (d["log_ns"][v] - d["stamp_ns"][v]).astype(float) * 1e-6
        print("\nMCAP log_time - MotorState.stamp:")
        print(f"  Median                                             : {np.median(latency_ms):.3f} ms")
        print(f"  Mean                                               : {np.mean(latency_ms):.3f} ms")
        print(f"  95th percentile absolute                          : {np.percentile(np.abs(latency_ms), 95):.3f} ms")
        print(f"  Minimum                                            : {np.min(latency_ms):.3f} ms")
        print(f"  Maximum                                            : {np.max(latency_ms):.3f} ms")

    print("\nLIMITATION: 左右position/velocityの個別読取時刻はMotorStateに存在しません。")
    print("USB問い合わせ順によるサンプル時刻差はvehicle_interface_node側で計測・記録が必要です。")
    return dt, valid, gap | nonpositive


def odom_impact(l: dict, r: dict, a: argparse.Namespace) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    circumference = 2 * math.pi * a.wheel_radius
    ldist = l["residual"] / a.gear_ratio * circumference
    rdist = r["residual"] / a.gear_ratio * circumference
    ds = 0.5 * (rdist + ldist)
    dyaw = (rdist - ldist) / a.tread_width
    anomaly = (
        (np.isfinite(ds) & (np.abs(ds) > a.odom_distance_threshold_m))
        | (np.isfinite(dyaw) & (np.abs(dyaw) > math.radians(a.odom_yaw_threshold_deg)))
    )

    heading("4. Estimated odometry impact of position/velocity residual")
    print(f"Per-sample thresholds: |translation| > {a.odom_distance_threshold_m * 1000:.3f} mm, "
          f"|yaw| > {a.odom_yaw_threshold_deg:.3f} deg")
    print(f"Threshold-exceeding samples                         : {np.count_nonzero(anomaly)}")
    print(f"Translation error MAE                               : {f(finite_metric(ds, 'mae') * 1000, 4)} mm")
    print(f"Translation error maximum absolute                  : {f(finite_metric(ds, 'max') * 1000, 4)} mm")
    print(f"Yaw error MAE                                       : {f(math.degrees(finite_metric(dyaw, 'mae')), 4)} deg")
    print(f"Yaw error maximum absolute                          : {f(math.degrees(finite_metric(dyaw, 'max')), 4)} deg")
    cumulative_ds = np.cumsum(np.where(np.isfinite(ds), ds, 0.0))
    cumulative_yaw = np.cumsum(np.where(np.isfinite(dyaw), dyaw, 0.0))
    print(f"Accumulated signed translation residual             : {cumulative_ds[-1] * 1000:.4f} mm")
    print(f"Accumulated signed yaw residual                     : {math.degrees(cumulative_yaw[-1]):.4f} deg")
    print("Note: reported RPSとの整合残差であり、外部基準に対する真の走行誤差ではありません。")
    return ds, dyaw, anomaly


def stop_check(d: dict, a: argparse.Namespace) -> None:
    mask = (
        np.isfinite(d["lcmd"]) & np.isfinite(d["rcmd"])
        & np.isfinite(d["lvel"]) & np.isfinite(d["rvel"])
        & (np.abs(d["lcmd"]) <= a.stop_cmd_rps)
        & (np.abs(d["rcmd"]) <= a.stop_cmd_rps)
        & (np.abs(d["lvel"]) <= a.stop_reported_rps)
        & (np.abs(d["rvel"]) <= a.stop_reported_rps)
    )
    selected = []
    for start, end in true_segments(mask):
        duration = (d["time_ns"][end] - d["time_ns"][start]) * 1e-9
        if duration >= a.stop_min_duration_sec:
            selected.append((start, end))

    heading("5. Stop-segment cumulative-position drift")
    print(f"Stop condition: |cmd| <= {a.stop_cmd_rps:g} RPS and |reported| <= {a.stop_reported_rps:g} RPS")
    print(f"Minimum duration                                      : {a.stop_min_duration_sec:g} s")
    print(f"Detected stop segments                                : {len(selected)}")
    if not selected:
        return

    circumference = 2 * math.pi * a.wheel_radius
    print("\n  #  start[s]   end[s]   dur[s]   L_net[turn]  R_net[turn]  L_p2p[turn]  R_p2p[turn]  ds[mm]  yaw[deg]")
    for n, (start, end) in enumerate(selected, 1):
        duration = (d["time_ns"][end] - d["time_ns"][start]) * 1e-9
        lp = d["lpos"][start:end + 1]
        rp = d["rpos"][start:end + 1]
        lnet = float(lp[-1] - lp[0])
        rnet = float(rp[-1] - rp[0])
        lp2p = float(np.nanmax(lp) - np.nanmin(lp))
        rp2p = float(np.nanmax(rp) - np.nanmin(rp))
        ldist = lnet / a.gear_ratio * circumference
        rdist = rnet / a.gear_ratio * circumference
        ds = 0.5 * (rdist + ldist)
        dyaw = (rdist - ldist) / a.tread_width
        print(f"  {n:2d} {d['t'][start]:9.3f} {d['t'][end]:8.3f} {duration:8.3f} "
              f"{lnet:12.6f} {rnet:12.6f} {lp2p:12.6f} {rp2p:12.6f} "
              f"{ds * 1000:7.3f} {math.degrees(dyaw):9.4f}")


def main() -> int:
    a = parse_args()
    try:
        validate_args(a)
        d = read_data(find_mcap_files(a.bag), a.topic)
        dt, timing_valid, timing_anomaly = timestamp_check(d, a)

        l = side_diagnostics(d["lpos"], d["lvel"], dt, timing_valid, a)
        r = side_diagnostics(d["rpos"], d["rvel"], dt, timing_valid, a)

        heading("1. Direct cumulative-position integrity checks")
        print(f"Physical derivative limit: {a.max_motor_rps * a.speed_margin:.3f} RPS")
        print_side("LEFT", l)
        print_side("RIGHT", r)
        direct = (
            timing_anomaly | l["nonfinite"] | r["nonfinite"]
            | l["jump"] | r["jump"] | l["stale"] | r["stale"]
            | l["direction"] | r["direction"] | l["reset"] | r["reset"]
        )
        print_events("Detailed direct-integrity events", direct, d, l, r, dt, a.top_events)

        heading("2. Position increment versus integrated reported velocity")
        print("Residual = actual delta pos_turns - trapezoid-integrated reported RPS")
        print_side("LEFT", l)
        print_side("RIGHT", r)

        heading("3. MAD-based residual anomaly detection")
        print(f"Threshold: |residual - median| > {a.mad_sigma:g} x robust_sigma")
        print(f"Robust sigma floor: {a.mad_sigma_floor_turns:g} turns")
        count("Left MAD anomalies", l["mad"])
        count("Right MAD anomalies", r["mad"])
        print_events("Detailed MAD events", l["mad"] | r["mad"], d, l, r, dt, a.top_events)

        _, _, odom_anomaly = odom_impact(l, r, a)
        print_events("Detailed odometry-impact events", odom_anomaly, d, l, r, dt, a.top_events)
        stop_check(d, a)

        heading("Overall assessment")
        severe = (
            l["nonfinite"] | r["nonfinite"] | l["jump"] | r["jump"]
            | l["stale"] | r["stale"] | l["direction"] | r["direction"]
            | l["reset"] | r["reset"]
        )
        print(f"Direct cumulative-position integrity events          : {np.count_nonzero(severe)}")
        print(f"MAD residual anomaly samples                         : {np.count_nonzero(l['mad'] | r['mad'])}")
        print(f"Odometry-impact threshold samples                    : {np.count_nonzero(odom_anomaly)}")
        print(f"Timestamp anomaly samples                            : {np.count_nonzero(timing_anomaly)}")

        if np.any(severe):
            print("RESULT: 通算回転数の直接異常を検出しました。詳細時刻を確認してください。")
        elif np.any(l["mad"] | r["mad"]):
            print("RESULT: 明確な飛び・固着・リセットはありませんが、統計的に異常な残差があります。")
        else:
            print("RESULT: 設定閾値内では通算回転数の直接異常を検出しませんでした。")
        print("同じエンコーダに共通して現れるposition/velocity双方の誤りは検出できないため、")
        print("最終確認には外部の回転数・走行距離基準が必要です。")

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"[ERROR] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
