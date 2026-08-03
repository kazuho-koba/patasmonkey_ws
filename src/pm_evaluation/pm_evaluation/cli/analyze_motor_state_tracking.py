#!/usr/bin/env python3
"""
/motor_stateだけを使い、モータ回転速度指令と実測速度を比較する。

使用するフィールド:
    stamp
    left_cmd_rps
    right_cmd_rps
    left_pos_turns
    right_pos_turns
    left_vel_rps
    right_vel_rps

解析内容:
    1. cmd_rpsとvel_rpsを直接比較する
    2. pos_turnsを時間差分し、独立な位置差分RPSを計算する
    3. vel_rpsと位置差分RPSの不一致から、位置値の飛びを疑う区間を検出する
    4. メッセージ時刻間隔の異常を検出する

出力:
    motor_rps_tracking.png
        左右の指令・実測・位置差分速度と誤差をまとめた1枚の図

    motor_rps_tracking.csv
        各サンプルの値と異常フラグ

使用例:
    ros2 run pm_evaluation analyze_motor_state_tracking \
      /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38/

    結果を上書き:
    ros2 run pm_evaluation analyze_motor_state_tracking \
      /path/to/bag --overwrite

    CSVを保存せず、図だけ出力:
    ros2 run pm_evaluation analyze_motor_state_tracking \
      /path/to/bag --no-csv
"""

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mcap_ros2.reader import read_ros2_messages


DEFAULT_TOPIC = "/motor_state"
DEFAULT_MAX_EXPECTED_RPS = 40.0
DEFAULT_ENCODER_MISMATCH_RPS = 5.0
DEFAULT_MEDIAN_WINDOW = 5

PLOT_NAME = "motor_rps_tracking.png"
CSV_NAME = "motor_rps_tracking.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare motor RPS commands, ODrive-reported RPS, "
            "and position-derived RPS from /motor_state."
        )
    )

    parser.add_argument(
        "bag",
        type=Path,
        help="Rosbag directory containing MCAP files, or one .mcap file.",
    )

    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=f"MotorState topic. Default: {DEFAULT_TOPIC}",
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help=(
            "Output directory. Default: "
            "<bag directory>/motor_state_tracking"
        ),
    )

    parser.add_argument(
        "--max-expected-rps",
        type=float,
        default=DEFAULT_MAX_EXPECTED_RPS,
        help=(
            "Expected maximum motor RPS. "
            "Used only for spike detection. Default: 40"
        ),
    )

    parser.add_argument(
        "--encoder-mismatch-rps",
        type=float,
        default=DEFAULT_ENCODER_MISMATCH_RPS,
        help=(
            "Flag a sample when abs(position-derived RPS - reported RPS) "
            "exceeds this value. Default: 5"
        ),
    )

    parser.add_argument(
        "--median-window",
        type=int,
        default=DEFAULT_MEDIAN_WINDOW,
        help=(
            "Odd rolling-median window for position-derived RPS. Default: 5"
        ),
    )

    parser.add_argument(
        "--no-csv",
        action="store_true",
        help="Do not save the CSV file.",
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing output files.",
    )

    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.max_expected_rps <= 0.0:
        raise ValueError("--max-expected-rps must be positive.")

    if args.encoder_mismatch_rps <= 0.0:
        raise ValueError("--encoder-mismatch-rps must be positive.")

    if args.median_window < 1 or args.median_window % 2 == 0:
        raise ValueError("--median-window must be a positive odd integer.")


def natural_mcap_sort_key(path: Path) -> Tuple[str, int]:
    """
    分割MCAPの末尾番号を数値として並べる。
    """
    stem = path.stem
    prefix, separator, suffix = stem.rpartition("_")

    if separator and suffix.isdigit():
        return prefix, int(suffix)

    return stem, -1


def find_mcap_files(path: Path) -> List[Path]:
    resolved = path.expanduser().resolve()

    if not resolved.exists():
        raise FileNotFoundError(f"Input does not exist: {resolved}")

    if resolved.is_file():
        if resolved.suffix.lower() != ".mcap":
            raise ValueError(f"Input file is not MCAP: {resolved}")

        return [resolved]

    files = sorted(
        resolved.glob("*.mcap"),
        key=natural_mcap_sort_key,
    )

    if not files:
        raise ValueError(f"No MCAP files found in: {resolved}")

    return files


def resolve_output_dir(
    bag_path: Path,
    output_dir: Path,
) -> Path:
    if output_dir is not None:
        resolved = output_dir.expanduser().resolve()
    else:
        source = bag_path.expanduser().resolve()

        if source.is_dir():
            resolved = source / "motor_state_tracking"
        else:
            resolved = (
                source.parent
                / f"{source.stem}_motor_state_tracking"
            )

    resolved.mkdir(
        parents=True,
        exist_ok=True,
    )

    return resolved


def check_output_policy(
    output_dir: Path,
    save_csv: bool,
    overwrite: bool,
) -> None:
    paths = [
        output_dir / PLOT_NAME,
    ]

    if save_csv:
        paths.append(output_dir / CSV_NAME)

    existing = [
        path
        for path in paths
        if path.exists()
    ]

    if existing and not overwrite:
        raise FileExistsError(
            "Output already exists. Use --overwrite:\n"
            + "\n".join(str(path) for path in existing)
        )


def stamp_to_ns(msg, fallback_ns: int) -> int:
    """
    MotorState.stampを優先し、無効ならMCAP log timeを使う。
    """
    try:
        sec = int(msg.stamp.sec)
        nanosec = int(msg.stamp.nanosec)

        if sec != 0 or nanosec != 0:
            return sec * 1_000_000_000 + nanosec
    except Exception:
        pass

    return int(fallback_ns)


def rolling_nanmedian(
    values: np.ndarray,
    window: int,
) -> np.ndarray:
    """
    SciPyを使わずにNaN対応の移動中央値を計算する。
    """
    if window <= 1:
        return values.copy()

    half = window // 2
    output = np.full_like(
        values,
        np.nan,
        dtype=float,
    )

    for index in range(len(values)):
        start = max(0, index - half)
        end = min(len(values), index + half + 1)

        subset = values[start:end]
        finite = subset[np.isfinite(subset)]

        if finite.size:
            output[index] = float(np.median(finite))

    return output


def finite_metric(
    first: np.ndarray,
    second: np.ndarray,
    mode: str,
) -> float:
    valid = np.isfinite(first) & np.isfinite(second)

    if not np.any(valid):
        return math.nan

    error = first[valid] - second[valid]

    if mode == "mae":
        return float(np.mean(np.abs(error)))

    if mode == "rmse":
        return float(np.sqrt(np.mean(error * error)))

    if mode == "max":
        return float(np.max(np.abs(error)))

    raise ValueError(f"Unknown metric: {mode}")


def interval_derivative(
    time_sec: np.ndarray,
    position_turns: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    累積回転位置を隣接サンプル差分し、RPSを計算する。

    Returns:
        derived_rps:
            先頭はNaN。以降はdelta_turns / delta_time。

        delta_turns:
            1サンプルごとの位置増分。

        delta_time:
            1サンプルごとの時刻差。
    """
    count = len(time_sec)

    derived_rps = np.full(
        count,
        np.nan,
        dtype=float,
    )
    delta_turns = np.full(
        count,
        np.nan,
        dtype=float,
    )
    delta_time = np.full(
        count,
        np.nan,
        dtype=float,
    )

    if count < 2:
        return derived_rps, delta_turns, delta_time

    dt = np.diff(time_sec)
    dp = np.diff(position_turns)

    valid = (
        np.isfinite(dt)
        & (dt > 0.0)
        & np.isfinite(dp)
    )

    derivative = np.full_like(
        dt,
        np.nan,
        dtype=float,
    )
    derivative[valid] = dp[valid] / dt[valid]

    derived_rps[1:] = derivative
    delta_turns[1:] = dp
    delta_time[1:] = dt

    return derived_rps, delta_turns, delta_time


def read_motor_state(
    mcap_files: Sequence[Path],
    topic: str,
) -> dict:
    """
    /motor_stateだけを読み込む。
    """
    timestamps_ns: List[int] = []
    left_cmd: List[float] = []
    right_cmd: List[float] = []
    left_pos: List[float] = []
    right_pos: List[float] = []
    left_vel: List[float] = []
    right_vel: List[float] = []

    required_fields = (
        "left_cmd_rps",
        "right_cmd_rps",
        "left_pos_turns",
        "right_pos_turns",
        "left_vel_rps",
        "right_vel_rps",
    )

    for mcap_path in mcap_files:
        print(f"[READ] {mcap_path}")

        for decoded in read_ros2_messages(
            mcap_path,
            topics={topic},
            log_time_order=True,
        ):
            if decoded.channel.topic != topic:
                continue

            msg = decoded.ros_msg

            missing = [
                field
                for field in required_fields
                if not hasattr(msg, field)
            ]

            if missing:
                raise AttributeError(
                    f"{topic} is missing fields: {missing}"
                )

            timestamps_ns.append(
                stamp_to_ns(
                    msg,
                    int(decoded.log_time_ns),
                )
            )
            left_cmd.append(float(msg.left_cmd_rps))
            right_cmd.append(float(msg.right_cmd_rps))
            left_pos.append(float(msg.left_pos_turns))
            right_pos.append(float(msg.right_pos_turns))
            left_vel.append(float(msg.left_vel_rps))
            right_vel.append(float(msg.right_vel_rps))

    if not timestamps_ns:
        raise RuntimeError(f"No messages found on {topic}")

    order = np.argsort(
        np.asarray(timestamps_ns, dtype=np.int64)
    )

    def ordered(values, dtype=float):
        return np.asarray(values, dtype=dtype)[order]

    timestamp_array = ordered(
        timestamps_ns,
        dtype=np.int64,
    )

    origin_ns = int(timestamp_array[0])

    return {
        "timestamp_ns": timestamp_array,
        "time_sec": (
            timestamp_array - origin_ns
        ).astype(float) * 1.0e-9,
        "left_cmd_rps": ordered(left_cmd),
        "right_cmd_rps": ordered(right_cmd),
        "left_pos_turns": ordered(left_pos),
        "right_pos_turns": ordered(right_pos),
        "left_vel_rps": ordered(left_vel),
        "right_vel_rps": ordered(right_vel),
    }


def add_event_lines(
    axes,
    time_sec: np.ndarray,
    event_mask: np.ndarray,
) -> None:
    """
    全サブプロットへ異常時刻を縦線で重ねる。
    """
    indices = np.flatnonzero(event_mask)

    for event_number, index in enumerate(indices):
        for axis_number, ax in enumerate(axes):
            label = None

            if event_number == 0 and axis_number == 0:
                label = "Encoder consistency anomaly"

            ax.axvline(
                time_sec[index],
                linestyle=":",
                linewidth=0.7,
                alpha=0.35,
                label=label,
            )


def save_plot(
    output_path: Path,
    data: dict,
    left_pos_rps_raw: np.ndarray,
    right_pos_rps_raw: np.ndarray,
    left_pos_rps_filtered: np.ndarray,
    right_pos_rps_filtered: np.ndarray,
    left_tracking_error: np.ndarray,
    right_tracking_error: np.ndarray,
    left_encoder_error: np.ndarray,
    right_encoder_error: np.ndarray,
    anomaly_mask: np.ndarray,
) -> None:
    """
    出力を1枚のPNGへ集約する。
    """
    fig, axes = plt.subplots(
        4,
        1,
        figsize=(15, 13),
        dpi=150,
        sharex=True,
    )

    time_sec = data["time_sec"]

    axes[0].plot(
        time_sec,
        data["left_cmd_rps"],
        label="Left command RPS",
    )
    axes[0].plot(
        time_sec,
        data["left_vel_rps"],
        label="Left reported RPS",
    )
    axes[0].plot(
        time_sec,
        left_pos_rps_raw,
        linewidth=0.7,
        alpha=0.35,
        label="Left position-derived RPS raw",
    )
    axes[0].plot(
        time_sec,
        left_pos_rps_filtered,
        label="Left position-derived RPS median",
    )
    axes[0].set_ylabel("Left RPS")
    axes[0].set_title(
        "Left motor: command, reported velocity, and position derivative"
    )
    axes[0].grid(True)
    axes[0].legend(loc="best")

    axes[1].plot(
        time_sec,
        data["right_cmd_rps"],
        label="Right command RPS",
    )
    axes[1].plot(
        time_sec,
        data["right_vel_rps"],
        label="Right reported RPS",
    )
    axes[1].plot(
        time_sec,
        right_pos_rps_raw,
        linewidth=0.7,
        alpha=0.35,
        label="Right position-derived RPS raw",
    )
    axes[1].plot(
        time_sec,
        right_pos_rps_filtered,
        label="Right position-derived RPS median",
    )
    axes[1].set_ylabel("Right RPS")
    axes[1].set_title(
        "Right motor: command, reported velocity, and position derivative"
    )
    axes[1].grid(True)
    axes[1].legend(loc="best")

    axes[2].plot(
        time_sec,
        left_tracking_error,
        label="Left reported - command",
    )
    axes[2].plot(
        time_sec,
        right_tracking_error,
        label="Right reported - command",
    )
    axes[2].axhline(
        0.0,
        linewidth=0.8,
    )
    axes[2].set_ylabel("Tracking error [RPS]")
    axes[2].set_title(
        "Motor velocity tracking error"
    )
    axes[2].grid(True)
    axes[2].legend(loc="best")

    axes[3].plot(
        time_sec,
        left_encoder_error,
        label="Left position-derived - reported",
    )
    axes[3].plot(
        time_sec,
        right_encoder_error,
        label="Right position-derived - reported",
    )
    axes[3].axhline(
        0.0,
        linewidth=0.8,
    )
    axes[3].set_ylabel("Consistency error [RPS]")
    axes[3].set_xlabel("Bag elapsed time [s]")
    axes[3].set_title(
        "Encoder consistency check"
    )
    axes[3].grid(True)
    axes[3].legend(loc="best")

    add_event_lines(
        axes,
        time_sec,
        anomaly_mask,
    )

    # 異常線の凡例を最上段へ確実に反映する。
    handles, labels = axes[0].get_legend_handles_labels()
    if handles:
        axes[0].legend(
            handles,
            labels,
            loc="best",
        )

    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def save_csv(
    output_path: Path,
    data: dict,
    left_pos_rps_raw: np.ndarray,
    right_pos_rps_raw: np.ndarray,
    left_pos_rps_filtered: np.ndarray,
    right_pos_rps_filtered: np.ndarray,
    delta_time: np.ndarray,
    left_tracking_error: np.ndarray,
    right_tracking_error: np.ndarray,
    left_encoder_error: np.ndarray,
    right_encoder_error: np.ndarray,
    left_mismatch: np.ndarray,
    right_mismatch: np.ndarray,
    timing_gap: np.ndarray,
    speed_spike: np.ndarray,
) -> None:
    headers = [
        "timestamp_ns",
        "bag_time_sec",
        "left_cmd_rps",
        "right_cmd_rps",
        "left_vel_rps",
        "right_vel_rps",
        "left_pos_turns",
        "right_pos_turns",
        "left_position_derived_rps_raw",
        "right_position_derived_rps_raw",
        "left_position_derived_rps_median",
        "right_position_derived_rps_median",
        "motor_state_dt_sec",
        "left_tracking_error_rps",
        "right_tracking_error_rps",
        "left_encoder_consistency_error_rps",
        "right_encoder_consistency_error_rps",
        "left_encoder_mismatch",
        "right_encoder_mismatch",
        "timing_gap",
        "speed_spike",
    ]

    columns = [
        data["timestamp_ns"],
        data["time_sec"],
        data["left_cmd_rps"],
        data["right_cmd_rps"],
        data["left_vel_rps"],
        data["right_vel_rps"],
        data["left_pos_turns"],
        data["right_pos_turns"],
        left_pos_rps_raw,
        right_pos_rps_raw,
        left_pos_rps_filtered,
        right_pos_rps_filtered,
        delta_time,
        left_tracking_error,
        right_tracking_error,
        left_encoder_error,
        right_encoder_error,
        left_mismatch.astype(int),
        right_mismatch.astype(int),
        timing_gap.astype(int),
        speed_spike.astype(int),
    ]

    with output_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(headers)

        for row_index in range(len(data["time_sec"])):
            row = []

            for column in columns:
                value = column[row_index]

                if isinstance(
                    value,
                    (
                        float,
                        np.floating,
                    ),
                ):
                    if math.isfinite(float(value)):
                        row.append(f"{float(value):.12g}")
                    else:
                        row.append("")
                else:
                    row.append(int(value))

            writer.writerow(row)


def print_metric(
    label: str,
    value: float,
    unit: str = "RPS",
) -> None:
    if math.isfinite(value):
        print(f"  {label:<43}: {value:10.4f} {unit}")
    else:
        print(f"  {label:<43}: unavailable")


def main() -> int:
    args = parse_args()

    try:
        validate_args(args)

        mcap_files = find_mcap_files(args.bag)
        output_dir = resolve_output_dir(
            args.bag,
            args.output_dir,
        )

        check_output_policy(
            output_dir,
            save_csv=not args.no_csv,
            overwrite=args.overwrite,
        )

        data = read_motor_state(
            mcap_files,
            args.topic,
        )

        (
            left_pos_rps_raw,
            left_delta_turns,
            left_dt,
        ) = interval_derivative(
            data["time_sec"],
            data["left_pos_turns"],
        )

        (
            right_pos_rps_raw,
            right_delta_turns,
            right_dt,
        ) = interval_derivative(
            data["time_sec"],
            data["right_pos_turns"],
        )

        delta_time = np.where(
            np.isfinite(left_dt),
            left_dt,
            right_dt,
        )

        median_dt = (
            float(np.nanmedian(delta_time))
            if np.any(np.isfinite(delta_time))
            else math.nan
        )

        timing_gap = np.zeros(
            len(data["time_sec"]),
            dtype=bool,
        )

        if math.isfinite(median_dt):
            timing_gap = (
                np.isfinite(delta_time)
                & (delta_time > 3.0 * median_dt)
            )

        nonpositive_dt = (
            np.isfinite(delta_time)
            & (delta_time <= 0.0)
        )
        timing_gap |= nonpositive_dt

        spike_limit = 1.5 * args.max_expected_rps

        raw_speed_spike = (
            (
                np.isfinite(left_pos_rps_raw)
                & (np.abs(left_pos_rps_raw) > spike_limit)
            )
            | (
                np.isfinite(right_pos_rps_raw)
                & (np.abs(right_pos_rps_raw) > spike_limit)
            )
            | (
                np.isfinite(data["left_vel_rps"])
                & (np.abs(data["left_vel_rps"]) > spike_limit)
            )
            | (
                np.isfinite(data["right_vel_rps"])
                & (np.abs(data["right_vel_rps"]) > spike_limit)
            )
        )

        # 明らかな時刻異常や速度スパイクは、中央値計算へ混ぜない。
        left_clean = left_pos_rps_raw.copy()
        right_clean = right_pos_rps_raw.copy()

        invalid_for_filter = (
            timing_gap
            | raw_speed_spike
        )

        left_clean[invalid_for_filter] = np.nan
        right_clean[invalid_for_filter] = np.nan

        left_pos_rps_filtered = rolling_nanmedian(
            left_clean,
            args.median_window,
        )
        right_pos_rps_filtered = rolling_nanmedian(
            right_clean,
            args.median_window,
        )

        left_tracking_error = (
            data["left_vel_rps"]
            - data["left_cmd_rps"]
        )
        right_tracking_error = (
            data["right_vel_rps"]
            - data["right_cmd_rps"]
        )

        left_encoder_error = (
            left_pos_rps_filtered
            - data["left_vel_rps"]
        )
        right_encoder_error = (
            right_pos_rps_filtered
            - data["right_vel_rps"]
        )

        left_mismatch = (
            np.isfinite(left_encoder_error)
            & (
                np.abs(left_encoder_error)
                > args.encoder_mismatch_rps
            )
        )
        right_mismatch = (
            np.isfinite(right_encoder_error)
            & (
                np.abs(right_encoder_error)
                > args.encoder_mismatch_rps
            )
        )

        anomaly_mask = (
            left_mismatch
            | right_mismatch
            | timing_gap
            | raw_speed_spike
        )

        plot_path = output_dir / PLOT_NAME

        save_plot(
            plot_path,
            data,
            left_pos_rps_raw,
            right_pos_rps_raw,
            left_pos_rps_filtered,
            right_pos_rps_filtered,
            left_tracking_error,
            right_tracking_error,
            left_encoder_error,
            right_encoder_error,
            anomaly_mask,
        )

        csv_path = output_dir / CSV_NAME

        if not args.no_csv:
            save_csv(
                csv_path,
                data,
                left_pos_rps_raw,
                right_pos_rps_raw,
                left_pos_rps_filtered,
                right_pos_rps_filtered,
                delta_time,
                left_tracking_error,
                right_tracking_error,
                left_encoder_error,
                right_encoder_error,
                left_mismatch,
                right_mismatch,
                timing_gap,
                raw_speed_spike,
            )

        sample_count = len(data["time_sec"])
        duration = (
            float(data["time_sec"][-1])
            if sample_count
            else 0.0
        )
        estimated_hz = (
            1.0 / median_dt
            if math.isfinite(median_dt) and median_dt > 0.0
            else math.nan
        )

        print("")
        print("=" * 72)
        print("MotorState tracking analysis")
        print("=" * 72)
        print(f"Samples                       : {sample_count}")
        print(f"Duration                      : {duration:.3f} s")

        if math.isfinite(estimated_hz):
            print(f"Estimated message rate        : {estimated_hz:.3f} Hz")

        print("")
        print("Command tracking: reported velocity - command")
        print_metric(
            "Left MAE",
            finite_metric(
                data["left_vel_rps"],
                data["left_cmd_rps"],
                "mae",
            ),
        )
        print_metric(
            "Left RMSE",
            finite_metric(
                data["left_vel_rps"],
                data["left_cmd_rps"],
                "rmse",
            ),
        )
        print_metric(
            "Left maximum absolute error",
            finite_metric(
                data["left_vel_rps"],
                data["left_cmd_rps"],
                "max",
            ),
        )
        print_metric(
            "Right MAE",
            finite_metric(
                data["right_vel_rps"],
                data["right_cmd_rps"],
                "mae",
            ),
        )
        print_metric(
            "Right RMSE",
            finite_metric(
                data["right_vel_rps"],
                data["right_cmd_rps"],
                "rmse",
            ),
        )
        print_metric(
            "Right maximum absolute error",
            finite_metric(
                data["right_vel_rps"],
                data["right_cmd_rps"],
                "max",
            ),
        )

        print("")
        print("Encoder consistency: position derivative - reported velocity")
        print_metric(
            "Left MAE",
            finite_metric(
                left_pos_rps_filtered,
                data["left_vel_rps"],
                "mae",
            ),
        )
        print_metric(
            "Left maximum absolute error",
            finite_metric(
                left_pos_rps_filtered,
                data["left_vel_rps"],
                "max",
            ),
        )
        print_metric(
            "Right MAE",
            finite_metric(
                right_pos_rps_filtered,
                data["right_vel_rps"],
                "mae",
            ),
        )
        print_metric(
            "Right maximum absolute error",
            finite_metric(
                right_pos_rps_filtered,
                data["right_vel_rps"],
                "max",
            ),
        )

        print("")
        print("Detected samples")
        print(f"  Left encoder mismatch       : {np.count_nonzero(left_mismatch)}")
        print(f"  Right encoder mismatch      : {np.count_nonzero(right_mismatch)}")
        print(f"  Timing gap/nonpositive dt   : {np.count_nonzero(timing_gap)}")
        print(f"  Speed spike                 : {np.count_nonzero(raw_speed_spike)}")
        print(f"  Any anomaly                 : {np.count_nonzero(anomaly_mask)}")

        print("")
        print(f"Plot                           : {plot_path}")

        if not args.no_csv:
            print(f"CSV                            : {csv_path}")

    except KeyboardInterrupt:
        print(
            "\n[INTERRUPTED]",
            file=sys.stderr,
        )
        return 130

    except Exception as error:
        print(
            f"[ERROR] {type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
