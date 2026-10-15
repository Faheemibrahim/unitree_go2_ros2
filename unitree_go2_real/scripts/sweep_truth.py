#!/usr/bin/env python3
"""Checkpoints for scoring: the true pose of the robot at every still moment of a bag,
measured from the room (offline).

  ros2 run unitree_go2_real sweep_truth.py <bag_dir>   -> <bag_dir>/truth.json

Every still is lined up with the FIRST still's walls (the matcher of odom_lidar_check.py,
self-tested there). The result is the robot's true pose at that still in the frame of
the first still, independent of the odometry and of how the robot was driven. A still
counts as a checkpoint only if its walls matched well (median < 5 cm, > 50 % of points
within 5 cm); weak ones are kept in the file, marked unusable.

truth.json: {"bag", "stills": [{"t0", "t1" (robot stamps), "true": [x, y, yaw] relative to
still 1, "robot_odom": [x, y, yaw] relative to still 1, "median", "matched", "usable"}]}
"""

import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_check import still_moments  # noqa: E402
from odom_lidar_check import align, apply, read, walls  # noqa: E402


def rel(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(-a[2]), math.sin(-a[2])
    return [c * dx - s * dy, s * dx + c * dy, (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi]


def main():
    bag = sys.argv[1]
    o, clouds = read(bag)
    t, x, y, yaw = o.T
    runs = still_moments(t, x, y, yaw)
    ct = np.array([c[0] for c in clouds])

    def still_walls(run):
        i0, i1 = run
        pts = [clouds[j][1] for j in np.flatnonzero((ct >= t[i0]) & (ct <= t[i1]))]
        c = np.array([x[i0:i1 + 1].mean(), y[i0:i1 + 1].mean()])
        return walls(np.vstack(pts), c)[0], c
    ref, _ = still_walls(runs[0])
    first = (x[runs[0][0]], y[runs[0][0]], yaw[runs[0][0]])
    out = []
    for k, run in enumerate(runs):
        i = run[0]
        c = np.array([x[i], y[i]])
        if k == 0:
            th, sh, med, inl = 0.0, np.zeros(2), 0.0, 1.0
        else:
            W, c = still_walls(run)
            th, sh, med, inl = align(W, ref, c)
        # align() returns a rotation about c plus a shift; move the robot's position the same way
        p = apply(np.array([[x[i], y[i]]]), c, th, sh)[0]
        true = rel(first, (p[0], p[1], yaw[i] + th))
        out.append({'t0': float(t[run[0]]), 't1': float(t[run[1]]), 'true': true,
                    'robot_odom': rel(first, (x[i], y[i], yaw[i])), 'median': float(med),
                    'matched': float(inl), 'usable': bool(med < 0.05 and inl > 0.5)})
        print(f'still {k + 1:2d} t {t[run[0]] - t[0]:6.1f} s: true ({true[0]:+.3f}, {true[1]:+.3f}, '
              f'{math.degrees(true[2]):+8.2f} deg)  match {med * 100:.1f} cm / {100 * inl:.0f}%  '
              f'{"checkpoint" if out[-1]["usable"] else "weak, not used"}')
    json.dump({'bag': os.path.basename(bag.rstrip('/')), 'stills': out}, open(os.path.join(bag, 'truth.json'), 'w'), indent=1)
    print(f'{sum(s["usable"] for s in out) - 1} checkpoints (besides still 1) -> {os.path.join(bag, "truth.json")}')


if __name__ == '__main__':
    main()
