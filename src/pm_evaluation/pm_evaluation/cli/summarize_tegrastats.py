#!/usr/bin/env python3
"""Summarize Jetson tegrastats logs captured during rosbag recording."""

import argparse
import re
import statistics
from pathlib import Path
from typing import List, Tuple


def percentile(values: List[float], fraction: float) -> float:
    """Return a nearest-rank value without adding a numerical dependency."""
    return sorted(values)[int(fraction * (len(values) - 1))]


def main() -> None:
    """Print CPU, RAM, GPU and temperature statistics for a tegrastats log."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tegrastats_log", type=Path)
    args = parser.parse_args()

    cpu_means: List[float] = []
    ram: List[Tuple[int, int]] = []
    gpu: List[int] = []
    cpu_temperatures: List[float] = []
    with args.tegrastats_log.open(errors="replace") as stream:
        for line in stream:
            per_core = [int(value) for value in re.findall(r"(\d+)%@", line)]
            if per_core:
                cpu_means.append(sum(per_core) / len(per_core))
            ram_match = re.search(r"RAM (\d+)/(\d+)MB", line)
            if ram_match:
                ram.append((int(ram_match.group(1)), int(ram_match.group(2))))
            gpu_match = re.search(r"GR3D_FREQ (\d+)%", line)
            if gpu_match:
                gpu.append(int(gpu_match.group(1)))
            temp_match = re.search(r"CPU@(\d+(?:\.\d+)?)C", line)
            if temp_match:
                cpu_temperatures.append(float(temp_match.group(1)))

    if not cpu_means:
        parser.error("no tegrastats samples found")
    print("samples={}".format(len(cpu_means)))
    print(
        "mean per-core CPU: mean={:.1f}% p95={:.1f}% max={:.1f}%".format(
            statistics.mean(cpu_means), percentile(cpu_means, 0.95),
            max(cpu_means),
        )
    )
    if ram:
        print("RAM: max={}/{} MB".format(max(used for used, _ in ram), ram[-1][1]))
    if gpu:
        print("GR3D: mean={:.1f}% max={}%;".format(statistics.mean(gpu), max(gpu)))
    if cpu_temperatures:
        print("CPU temperature: max={:.1f} C".format(max(cpu_temperatures)))


if __name__ == "__main__":
    main()
