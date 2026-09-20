#!/usr/bin/env python3
"""Compare direct wheel integration using three headings, without an EKF.

Usage: python3 tools/diagnose_wit_yaw.py RESULTS_DIRECTORY
Reads CSVs from compare_odometry_sources; writes yaw_diagnosis.png/json.
Gyro integration assumes planar motion, with no bias or magnetic correction.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path)
    out = parser.parse_args().results
    data = {k: np.loadtxt(out / (k + '.csv'), delimiter=',', skiprows=1)
            for k in ['raw_wheel', 'raw_vio', 'wheel_imu', 'imu']}
    summary = json.loads((out / 'summary.json').read_text())
    start, end = summary['common_start'], summary['common_end']
    w, v, e, i = [data[k] for k in ['raw_wheel', 'raw_vio', 'wheel_imu', 'imu']]
    gyro = np.r_[0, np.cumsum(np.diff(i[:, 0]) * (i[:-1, 2] + i[1:, 2]) / 2)]
    headings = {'Wit orientation yaw': (i[:, 0], np.unwrap(i[:, 1])),
                'Wit gyro z integral': (i[:, 0], gyro),
                'VIO orientation yaw': (v[:, 0], np.unwrap(v[:, 4]))}
    headings = {k: (t, a - np.interp(start, t, a)) for k, (t, a) in headings.items()}
    positions = {k: np.zeros(2) for k in headings}
    paths = {k: [np.zeros(2)] for k in headings}
    times = [start]
    for n in range(1, len(w)):
        lo, hi = max(start, w[n-1, 0]), min(end, w[n, 0])
        if hi <= lo:
            continue
        # Wheel vx describes displacement during the preceding encoder interval.
        t = np.linspace(lo, hi, max(2, int(np.ceil((hi-lo)/.005)) + 1))
        mid, dt = (t[:-1] + t[1:])/2, np.diff(t)
        for k, (ht, hy) in headings.items():
            angle = np.interp(mid, ht, hy)
            positions[k] += w[n, 5] * np.array([
                np.dot(dt, np.cos(angle)), np.dot(dt, np.sin(angle))])
            paths[k].append(positions[k].copy())
        times.append(hi)
    ep = np.column_stack([np.interp(times, e[:, 0], e[:, col]) for col in [1, 2]])
    ey = np.interp(start, e[:, 0], np.unwrap(e[:, 4]))
    rotation = np.array([[np.cos(ey), -np.sin(ey)], [np.sin(ey), np.cos(ey)]])
    ep = (ep - ep[0]) @ rotation
    paths = {k: np.array(a) for k, a in paths.items()}
    report = {'note': 'All paths use the SAME wheel displacements; only heading changes. '
              'Initial position/yaw normalized, no map or GNSS fitting. Gyro assumes 2D motion.',
              'common_start': start, 'common_end': end,
              'imu_topic': '/wit/imu', 'endpoint_distance_m': {},
              'direct_wit_yaw_vs_ekf_rmse_m': float(np.sqrt(np.mean(np.sum(
                  (paths['Wit orientation yaw'] - ep)**2, axis=1))))}
    for k, a in paths.items():
        report['endpoint_distance_m'][k] = float(np.linalg.norm(a[-1]))
    report['endpoint_distance_m']['wheel_imu EKF'] = float(np.linalg.norm(ep[-1]))
    for label, t in [('first_recorded', i[0, 0]), ('common_start', start), ('end', end)]:
        deg = np.degrees(np.interp(t, i[:, 0], np.unwrap(i[:, 1])))
        report['wit_yaw_deg_' + label] = float((deg+180) % 360-180)
    report['heading_window_medians_deg'] = {}
    for lo, hi in [(30, 50), (80, 100)]:
        report['heading_window_medians_deg'][f'{lo}-{hi}s'] = {
            k: float(np.degrees(np.median(y[(t >= start+lo) & (t <= start+hi)])))
            for k, (t, y) in headings.items()}
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    for k, a in paths.items():
        axes[0].plot(a[:, 0], a[:, 1], label=k)
        axes[0].plot(a[-1, 0], a[-1, 1], 'o', color=axes[0].lines[-1].get_color())
    axes[0].plot(ep[:, 0], ep[:, 1], 'k--', lw=1, label='wheel_imu EKF')
    axes[0].plot(0, 0, 'k*', ms=12, label='Common start')
    axes[0].set(xlabel='Initial body-forward axis [m]', ylabel='Initial body-left axis [m]',
                title='Same wheel displacements, different heading sources', aspect='equal')
    for k, (t, a) in headings.items():
        mask = (t >= start) & (t <= end)
        axes[1].plot(t[mask]-start, np.degrees(a[mask]), label=k)
    axes[1].set(xlabel='Time since common start [s]', ylabel='Unwrapped relative yaw [deg]',
                title='Wit orientation vs its own gyro vs VIO')
    for ax in axes:
        ax.legend(fontsize=8)
        ax.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(out / 'yaw_diagnosis.png', dpi=160)
    (out / 'yaw_diagnosis.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
