#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser(
        description='Generate the hover position drift plot from position.csv')
    ap.add_argument('--data-dir', default=str(Path.home() / '.px4_viz'))
    ap.add_argument('--output', default='hover_drift.png')
    args = ap.parse_args()

    csv_path = Path(args.data_dir) / 'position.csv'
    if not csv_path.is_file():
        print(f'ERROR: {csv_path} not found')
        return 1

    rows = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        print('ERROR: no position data recorded')
        return 1

    t0 = float(rows[0]['t'])
    x0 = float(rows[0]['x_ned'])
    y0 = float(rows[0]['y_ned'])

    t = [(float(r['t']) - t0) for r in rows]
    dx = [float(r['x_ned']) - x0 for r in rows]
    dy = [float(r['y_ned']) - y0 for r in rows]
    dist = [math.hypot(dx[i], dy[i]) for i in range(len(rows))]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    axes[0].plot(dx, dy, lw=1.0, label='hover path')
    axes[0].plot(0, 0, 'go', markersize=8, label='start')
    axes[0].plot(dx[-1], dy[-1], 'ro', markersize=8, label='end')
    r = max(max(dist), 0.1)
    axes[0].add_patch(plt.Circle((0, 0), r, fill=False, ls='--', color='gray', alpha=0.7))
    axes[0].set_aspect('equal', adjustable='datalim')
    axes[0].set_xlabel('drift north (m)')
    axes[0].set_ylabel('drift east (m)')
    axes[0].set_title('Hover Position Drift (NED)')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best')

    axes[1].plot(t, dist, lw=1.0, color='tab:orange')
    axes[1].set_xlabel('time (s)')
    axes[1].set_ylabel('drift distance (m)')
    axes[1].set_title('Hover Drift Distance vs Time')
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    plt.close(fig)

    mean_d = sum(dist) / len(dist)
    rms_d = math.sqrt(sum(d * d for d in dist) / len(dist))
    print(f'duration={t[-1]:.1f}s samples={len(rows)}')
    print(f'drift max={max(dist):.3f}m mean={mean_d:.3f}m rms={rms_d:.3f}m')
    print(f'end offset: N={dx[-1]:.3f}m E={dy[-1]:.3f}m')
    print(f'plot saved to {args.output}')


if __name__ == '__main__':
    main()
