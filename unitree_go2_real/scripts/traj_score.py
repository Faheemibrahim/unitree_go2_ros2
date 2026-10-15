#!/usr/bin/env python3
"""Score a trajectory against the checkpoints in <bag>/truth.json (offline).

  ros2 run unitree_go2_real traj_score.py <bag_dir> <traj.tum> [--frame body|lidar] [--name NAME]

The trajectory (TUM: t x y z qx qy qz qw, robot stamps) is read at every still's start time,
turned into the body's planar pose (x, y, yaw) - for --frame lidar through the fixed mount
base_link -> utlidar_lidar - and expressed relative to still 1, exactly like the truth.
Per usable checkpoint: position error and heading error. Summary: RMS of both, and the error
at the last checkpoint (the loop). Prints one line per checkpoint and a summary line; with
--json, writes the summary to <traj>.score.json.
"""

import argparse
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation

MOUNT = np.eye(4)
MOUNT[:3, :3] = Rotation.from_euler('xyz', [0, 2.8782, 0]).as_matrix()
MOUNT[:3, 3] = [0.28945, 0, -0.046825]


def rel(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(-a[2]), math.sin(-a[2])
    return [c * dx - s * dy, s * dx + c * dy, (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi]


def planar(T):
    x = T[:3, 0]                       # body x axis in the world
    return T[0, 3], T[1, 3], math.atan2(x[1], x[0])


def load_tum(path, frame):
    A = np.loadtxt(path)
    A = A[np.argsort(A[:, 0])]
    inv = np.linalg.inv(MOUNT)

    def pose(t):
        i = int(np.clip(np.searchsorted(A[:, 0], t), 0, len(A) - 1))
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(A[i, 4:8]).as_matrix()
        T[:3, 3] = A[i, 1:4]
        return planar(T @ inv if frame == 'lidar' else T), abs(A[i, 0] - t)
    return pose, A[0, 0], A[-1, 0]


def score(truth, pose):
    st = truth['stills']
    p0, _ = pose(st[0]['t0'])
    rows = []
    for k, s in enumerate(st[1:], start=2):
        if not s['usable']:
            continue
        p, gap = pose(s['t0'])
        e = rel(p0, p)
        tr = s['true']
        rows.append((k, math.hypot(e[0] - tr[0], e[1] - tr[1]),
                     math.degrees(abs((e[2] - tr[2] + math.pi) % (2 * math.pi) - math.pi)), gap))
    R = np.array([r[1:3] for r in rows])
    return rows, {'rms_pos': float(np.sqrt(np.mean(R[:, 0] ** 2))), 'rms_yaw': float(np.sqrt(np.mean(R[:, 1] ** 2))),
                  'end_pos': float(R[-1, 0]), 'end_yaw': float(R[-1, 1]), 'n': len(rows)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('traj')
    ap.add_argument('--frame', choices=['body', 'lidar'], default='body')
    ap.add_argument('--name', default=None)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    truth = json.load(open(f'{a.bag}/truth.json'))
    pose, t0, t1 = load_tum(a.traj, a.frame)
    rows, summ = score(truth, pose)
    name = a.name or a.traj
    for k, ep, ey, gap in rows:
        print(f'  {name}: checkpoint still {k}: position error {ep:.3f} m, heading error {ey:.2f} deg'
              + ('' if gap < 0.1 else f'  (nearest pose {gap:.2f} s away)'))
    print(f'{name}: {summ["n"]} checkpoints, RMS {summ["rms_pos"]:.3f} m / {summ["rms_yaw"]:.2f} deg, '
          f'last {summ["end_pos"]:.3f} m / {summ["end_yaw"]:.2f} deg')
    if a.json:
        json.dump(summ, open(a.traj + '.score.json', 'w'))


if __name__ == '__main__':
    main()
