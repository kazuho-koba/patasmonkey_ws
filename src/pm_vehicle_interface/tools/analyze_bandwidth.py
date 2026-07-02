#!/usr/bin/env python3

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


REQUIRED_COLUMNS = [
    "bandwidth",
    "axis_index",
    "segment_index",
    "t",
    "cmd_vel",
    "input_vel",
    "vel_estimate",
    "pos_estimate",
    "iq_measured",
    "iq_setpoint",
    "vbus_voltage",
    "axis_error",
    "motor_error",
    "encoder_error",
    "controller_error",
]


def load_csv(path: Path) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(f"CSV not found: {path}")

    df = pd.read_csv(path)

    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"{path} is missing columns: {missing}")

    # 数値化。読めない値は NaN にする。
    for col in REQUIRED_COLUMNS:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # ファイル名も記録しておく
    df["source_file"] = path.name

    return df


def add_derived_columns(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()

    df["vel_error"] = df["cmd_vel"] - df["vel_estimate"]
    df["abs_vel_error"] = df["vel_error"].abs()
    df["abs_iq_measured"] = df["iq_measured"].abs()
    df["abs_iq_setpoint"] = df["iq_setpoint"].abs()

    # 何らかのエラーが出ているか
    df["has_error"] = (
        (df["axis_error"].fillna(0) != 0)
        | (df["motor_error"].fillna(0) != 0)
        | (df["encoder_error"].fillna(0) != 0)
        | (df["controller_error"].fillna(0) != 0)
    )

    return df


def summarize_by_bandwidth(df: pd.DataFrame) -> pd.DataFrame:
    rows = []

    for bw, g in df.groupby("bandwidth"):
        valid = g.dropna(subset=["cmd_vel", "vel_estimate", "iq_measured"])

        if len(valid) == 0:
            rows.append({
                "bandwidth": bw,
                "n_samples": len(g),
                "vel_rmse": np.nan,
                "vel_mae": np.nan,
                "vel_std": np.nan,
                "iq_abs_mean": np.nan,
                "iq_std": np.nan,
                "iq_abs_max": np.nan,
                "vbus_min": np.nan,
                "error_samples": int(g["has_error"].sum()),
                "error_ratio": float(g["has_error"].mean()),
                "max_axis_error": int(np.nanmax(g["axis_error"])) if g["axis_error"].notna().any() else -1,
                "max_motor_error": int(np.nanmax(g["motor_error"])) if g["motor_error"].notna().any() else -1,
                "max_encoder_error": int(np.nanmax(g["encoder_error"])) if g["encoder_error"].notna().any() else -1,
                "max_controller_error": int(np.nanmax(g["controller_error"])) if g["controller_error"].notna().any() else -1,
            })
            continue

        vel_error = valid["vel_error"]

        rows.append({
            "bandwidth": bw,
            "n_samples": len(g),
            "vel_rmse": float(np.sqrt(np.nanmean(vel_error ** 2))),
            "vel_mae": float(np.nanmean(np.abs(vel_error))),
            "vel_std": float(np.nanstd(valid["vel_estimate"])),
            "iq_abs_mean": float(np.nanmean(valid["abs_iq_measured"])),
            "iq_std": float(np.nanstd(valid["iq_measured"])),
            "iq_abs_max": float(np.nanmax(valid["abs_iq_measured"])),
            "vbus_min": float(np.nanmin(valid["vbus_voltage"])),
            "error_samples": int(g["has_error"].sum()),
            "error_ratio": float(g["has_error"].mean()),
            "max_axis_error": int(np.nanmax(g["axis_error"])),
            "max_motor_error": int(np.nanmax(g["motor_error"])),
            "max_encoder_error": int(np.nanmax(g["encoder_error"])),
            "max_controller_error": int(np.nanmax(g["controller_error"])),
        })

    summary = pd.DataFrame(rows)
    summary = summary.sort_values("bandwidth").reset_index(drop=True)
    return summary


def summarize_steady_segments(df: pd.DataFrame, trim_start=0.3, trim_end=0.2) -> pd.DataFrame:
    """
    各segmentの先頭/末尾を除外して定常区間っぽい部分だけを集計する。
    step応答直後の立ち上がりを除外したいので、全体集計より制御振動の評価に向く。
    """
    rows = []

    for (bw, seg), g in df.groupby(["bandwidth", "segment_index"]):
        g = g.sort_values("t").copy()

        t0 = float(g["t"].min())
        t1 = float(g["t"].max())

        steady = g[(g["t"] >= t0 + trim_start) & (g["t"] <= t1 - trim_end)]

        if len(steady) < 5:
            continue

        cmd = float(np.nanmedian(steady["cmd_vel"]))

        rows.append({
            "bandwidth": bw,
            "segment_index": seg,
            "cmd_vel": cmd,
            "n_samples": len(steady),
            "vel_mean": float(np.nanmean(steady["vel_estimate"])),
            "vel_std": float(np.nanstd(steady["vel_estimate"])),
            "vel_error_mean": float(np.nanmean(steady["vel_error"])),
            "vel_error_rmse": float(np.sqrt(np.nanmean(steady["vel_error"] ** 2))),
            "iq_mean": float(np.nanmean(steady["iq_measured"])),
            "iq_std": float(np.nanstd(steady["iq_measured"])),
            "iq_abs_mean": float(np.nanmean(steady["abs_iq_measured"])),
            "iq_abs_max": float(np.nanmax(steady["abs_iq_measured"])),
            "error_samples": int(steady["has_error"].sum()),
            "error_ratio": float(steady["has_error"].mean()),
        })

    return pd.DataFrame(rows).sort_values(["bandwidth", "segment_index"]).reset_index(drop=True)


def plot_velocity_response(df: pd.DataFrame, outdir: Path, title_prefix: str):
    for bw, g in df.groupby("bandwidth"):
        g = g.sort_values("t")

        t = g["t"].to_numpy()
        cmd_vel = g["cmd_vel"].to_numpy()
        vel_estimate = g["vel_estimate"].to_numpy()

        plt.figure(figsize=(10, 5))
        plt.plot(t, cmd_vel, label="cmd_vel")
        plt.plot(t, vel_estimate, label="vel_estimate")
        plt.xlabel("time [s]")
        plt.ylabel("velocity [turn/s]")
        plt.title(f"{title_prefix}: velocity response, bandwidth={bw}")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        path = outdir / f"{title_prefix}_velocity_bw_{bw:g}.png"
        plt.savefig(path, dpi=150)
        plt.close()


def plot_current_response(df: pd.DataFrame, outdir: Path, title_prefix: str):
    for bw, g in df.groupby("bandwidth"):
        g = g.sort_values("t")

        t = g["t"].to_numpy()
        iq_measured = g["iq_measured"].to_numpy()
        iq_setpoint = g["iq_setpoint"].to_numpy()

        plt.figure(figsize=(10, 5))
        plt.plot(t, iq_measured, label="Iq_measured")
        plt.plot(t, iq_setpoint, label="Iq_setpoint")
        plt.xlabel("time [s]")
        plt.ylabel("current [A]")
        plt.title(f"{title_prefix}: current response, bandwidth={bw}")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        path = outdir / f"{title_prefix}_current_bw_{bw:g}.png"
        plt.savefig(path, dpi=150)
        plt.close()


def plot_error_response(df: pd.DataFrame, outdir: Path, title_prefix: str):
    for bw, g in df.groupby("bandwidth"):
        g = g.sort_values("t")

        t = g["t"].to_numpy()

        plt.figure(figsize=(10, 5))
        plt.plot(t, g["axis_error"].to_numpy(), label="axis_error")
        plt.plot(t, g["motor_error"].to_numpy(), label="motor_error")
        plt.plot(t, g["encoder_error"].to_numpy(), label="encoder_error")
        plt.plot(t, g["controller_error"].to_numpy(), label="controller_error")
        plt.xlabel("time [s]")
        plt.ylabel("error code")
        plt.title(f"{title_prefix}: error fields, bandwidth={bw}")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        path = outdir / f"{title_prefix}_errors_bw_{bw:g}.png"
        plt.savefig(path, dpi=150)
        plt.close()


def plot_summary(summary: pd.DataFrame, outdir: Path, title_prefix: str):
    if summary.empty:
        return

    x = summary["bandwidth"].to_numpy()

    plt.figure(figsize=(8, 5))
    plt.plot(x, summary["vel_rmse"].to_numpy(), marker="o", label="vel_rmse")
    plt.plot(x, summary["vel_mae"].to_numpy(), marker="o", label="vel_mae")
    plt.xlabel("encoder bandwidth")
    plt.ylabel("velocity error [turn/s]")
    plt.title(f"{title_prefix}: velocity error summary")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / f"{title_prefix}_summary_velocity_error.png", dpi=150)
    plt.close()

    plt.figure(figsize=(8, 5))
    plt.plot(x, summary["iq_abs_mean"].to_numpy(), marker="o", label="mean |Iq_measured|")
    plt.plot(x, summary["iq_abs_max"].to_numpy(), marker="o", label="max |Iq_measured|")
    plt.xlabel("encoder bandwidth")
    plt.ylabel("current [A]")
    plt.title(f"{title_prefix}: current summary")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / f"{title_prefix}_summary_current.png", dpi=150)
    plt.close()

    plt.figure(figsize=(8, 5))
    plt.plot(x, summary["error_ratio"].to_numpy(), marker="o", label="error_ratio")
    plt.xlabel("encoder bandwidth")
    plt.ylabel("error sample ratio")
    plt.title(f"{title_prefix}: error ratio")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / f"{title_prefix}_summary_error_ratio.png", dpi=150)
    plt.close()


def analyze_one_csv(csv_path: Path, outdir: Path):
    print(f"\n=== Loading {csv_path} ===")

    df = load_csv(csv_path)
    df = add_derived_columns(df)

    # axis情報
    axis_values = sorted(df["axis_index"].dropna().unique().tolist())
    axis_label = f"axis{int(axis_values[0])}" if len(axis_values) == 1 else "multi_axis"
    title_prefix = csv_path.stem

    print(f"axis: {axis_values}")
    print(f"bandwidths: {sorted(df['bandwidth'].dropna().unique().tolist())}")
    print(f"samples: {len(df)}")

    csv_outdir = outdir / csv_path.stem
    csv_outdir.mkdir(parents=True, exist_ok=True)

    summary = summarize_by_bandwidth(df)
    steady_summary = summarize_steady_segments(df)

    summary_path = csv_outdir / f"{csv_path.stem}_summary_by_bandwidth.csv"
    steady_path = csv_outdir / f"{csv_path.stem}_steady_segments.csv"

    summary.to_csv(summary_path, index=False)
    steady_summary.to_csv(steady_path, index=False)

    print("\nSummary by bandwidth:")
    print(summary.to_string(index=False))

    print(f"\nSaved summary: {summary_path}")
    print(f"Saved steady segment summary: {steady_path}")

    plot_velocity_response(df, csv_outdir, title_prefix)
    plot_current_response(df, csv_outdir, title_prefix)
    plot_error_response(df, csv_outdir, title_prefix)
    plot_summary(summary, csv_outdir, title_prefix)

    print(f"Saved plots under: {csv_outdir}")

    return df, summary, steady_summary


def compare_axes(all_summaries, outdir: Path):
    """
    axis0/axis1 のsummaryが両方ある場合に、横並び比較プロットを作る。
    """
    if len(all_summaries) < 2:
        return

    combined = []
    for name, summary in all_summaries:
        s = summary.copy()
        s["source"] = name
        combined.append(s)

    combined = pd.concat(combined, ignore_index=True)

    compare_path = outdir / "combined_summary_by_bandwidth.csv"
    combined.to_csv(compare_path, index=False)

    metrics = [
        ("vel_rmse", "velocity RMSE [turn/s]"),
        ("iq_abs_mean", "mean |Iq_measured| [A]"),
        ("iq_abs_max", "max |Iq_measured| [A]"),
        ("error_ratio", "error sample ratio"),
    ]

    for metric, ylabel in metrics:
        plt.figure(figsize=(8, 5))

        for source, g in combined.groupby("source"):
            g = g.sort_values("bandwidth")
            plt.plot(
                g["bandwidth"].to_numpy(),
                g[metric].to_numpy(),
                marker="o",
                label=source,
            )

        plt.xlabel("encoder bandwidth")
        plt.ylabel(ylabel)
        plt.title(f"axis comparison: {metric}")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(outdir / f"compare_{metric}.png", dpi=150)
        plt.close()

    print(f"\nSaved combined summary: {compare_path}")
    print(f"Saved comparison plots under: {outdir}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze ODrive encoder bandwidth test CSV files."
    )

    parser.add_argument(
        "--input-dir",
        type=str,
        default="bandwidth_results",
        help="Directory containing bandwidth test CSV files. Default: bandwidth_results",
    )

    parser.add_argument(
        "--pattern",
        type=str,
        default="*.csv",
        help="CSV filename pattern. Default: *.csv",
    )

    parser.add_argument(
        "--output-dir",
        type=str,
        default="bandwidth_analysis",
        help="Directory to save analysis results. Default: bandwidth_analysis",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    script_dir = Path(__file__).resolve().parent
    input_dir = Path(args.input_dir)

    # 相対パス指定なら、このスクリプトがある tools/ からの相対パスとして扱う
    if not input_dir.is_absolute():
        input_dir = script_dir / input_dir

    outdir = Path(args.output_dir)
    if not outdir.is_absolute():
        outdir = script_dir / outdir

    outdir.mkdir(parents=True, exist_ok=True)

    csv_files = sorted(input_dir.glob(args.pattern))

    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in {input_dir} with pattern {args.pattern}")

    print(f"Input dir: {input_dir}")
    print(f"Output dir: {outdir}")
    print("CSV files:")
    for p in csv_files:
        print(f"  - {p.name}")

    all_summaries = []

    for csv_path in csv_files:
        _, summary, _ = analyze_one_csv(csv_path, outdir)
        all_summaries.append((csv_path.stem, summary))

    compare_axes(all_summaries, outdir)

    print("\nDone.")


if __name__ == "__main__":
    main()