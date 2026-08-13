#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser(
        description='Generate the vibration curve plot from vibration.csv')
    ap.add_argument('--data-dir', default=str(Path.home() / '.px4_viz'))
    ap.add_argument('--output', default='vibration.png')
    args = ap.parse_args()

    csv_path = Path(args.data_dir) / 'vibration.csv'
    if not csv_path.is_file():
        print(f'ERROR: {csv_path} not found')
        return 1

    rows = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        print('ERROR: no vibration data recorded')
        return 1

    t0 = float(rows[0]['t'])
    t = [(float(r['t']) - t0) for r in rows]
    vib_x = [float(r['vib_x']) for r in rows]
    vib_y = [float(r['vib_y']) for r in rows]
    vib_z = [float(r['vib_z']) for r in rows]
    clip = [float(r['clip_0']) + float(r['clip_1']) + float(r['clip_2']) for r in rows]

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, data, label in ((axes[0], vib_x, 'X (gyro coning)'),
                            (axes[1], vib_y, 'Y (gyro high-freq)'),
                            (axes[2], vib_z, 'Z (accel high-freq)')):
        ax.plot(t, data, lw=1.0)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)

    axes[0].set_title('IMU Vibration (MAVLink VIBRATION)')
    axes[2].set_xlabel('time (s)')
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    plt.close(fig)

    stats = f'vib_x mean={sum(vib_x)/len(vib_x):.3f} max={max(vib_x):.3f}\n' \
            f'vib_y mean={sum(vib_y)/len(vib_y):.3f} max={max(vib_y):.3f}\n' \
            f'vib_z mean={sum(vib_z)/len(vib_z):.3f} max={max(vib_z):.3f}\n' \
            f'total accel clips={max(clip):.0f}'
    print(stats)
    print(f'plot saved to {args.output}')


if __name__ == '__main__':
    main()
