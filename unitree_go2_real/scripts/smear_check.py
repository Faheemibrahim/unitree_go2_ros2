#!/usr/bin/env python3
"""How much does merging lidar messages smear the walls? (offline, from a bag)

  ros2 run unitree_go2_real smear_check.py <bag_dir> [--yaw-scale 0.9814] [--every 6]

The 2D scan is built from the last W seconds of messages, each placed with odometry. If
odometry were perfect, a wall seen by older messages would land exactly on the same wall
seen by the newest message. The extra spread is the smear.

Per sampled scan: straight wall pieces are found in the NEWEST message alone (points in the
scan band, crop box applied, 0.5-6 m away; sequential RANSAC, a piece needs >= 20 points
within 3 cm and >= 0.5 m length). For each piece and each OLDER message in the window, the
signed distance of that message's points on the piece to the line (along the segment,
within +-0.15 m, then recentred three times on the points within 6 cm of the estimate,
>= 5 points): how far odometry misplaced that message across the wall. Smear = the median of those |offsets| over all pieces, messages and scans;
also the 90th percentile. The newest message's own spread is shown as the noise floor.

Self-test first: the older points are moved by a known 5 cm across each line; the measure
must report 5 cm (+-1). (Rejected versions: median distance of all older points within
+-0.15 m read a 5 cm shift as 1.3 cm; median signed offset within a fixed +-0.10 m read it as
4.0 cm, clipped by the window. The recentred estimate follows the wall.)

Also prints coverage (front +-90 deg bins with a return) for the same windows, so the two
can be read together, and the walking speed range of the bag.
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odom_lidar_check import correct, read  # noqa: E402

WINDOWS = (0.1, 0.2, 0.3, 0.5, 1.0)
RNG = np.random.default_rng(0)


def lines(P, max_lines=4):
    """Sequential RANSAC on 2D points: [(point, direction, normal, t_min, t_max, inlier_idx)]."""
    out, left = [], np.arange(len(P))
    for _ in range(max_lines):
        if len(left) < 20:
            break
        Q = P[left]
        best = None
        for _ in range(150):
            a, b = Q[RNG.choice(len(Q), 2, replace=False)]
            d = b - a
            L = np.hypot(*d)
            if L < 0.1:
                continue
            d /= L
            n = np.array([-d[1], d[0]])
            inl = np.abs((Q - a) @ n) < 0.03
            if best is None or inl.sum() > best[1].sum():
                best = (a, inl, d, n)
        if best is None or best[1].sum() < 20:
            break
        a, inl, d, n = best
        # refit on inliers (PCA)
        pts = Q[inl]
        c = pts.mean(0)
        u, s, vt = np.linalg.svd(pts - c)
        d = vt[0]
        n = np.array([-d[1], d[0]])
        t = (pts - c) @ d
        if t.max() - t.min() >= 0.5:
            out.append((c, d, n, t.min(), t.max(), left[inl]))
        left = left[~inl]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--yaw-scale', type=float, default=0.9814)
    ap.add_argument('--every', type=int, default=6)
    ap.add_argument('--config', default=None)
    args = ap.parse_args()
    if args.config is None:
        from ament_index_python.packages import get_package_share_directory
        args.config = os.path.join(get_package_share_directory('unitree_go2_real'),
                                   'config', 'pointcloud_to_laserscan.yaml')
    cfg = yaml.safe_load(open(args.config))
    crop, band = cfg['body_crop']['ros__parameters'], cfg['pointcloud_to_laserscan']['ros__parameters']
    lo, hi = band['min_height'], band['max_height']

    o, clouds = read(args.bag)
    o, clouds = correct(o, clouds, args.yaw_scale)
    t = o[:, 0]
    ct = np.array([c[0] for c in clouds])
    k = max(1, int(round(0.2 / np.median(np.diff(t)))))
    v = np.hypot(o[k:, 1] - o[:-k, 1], o[k:, 2] - o[:-k, 2]) / (t[k:] - t[:-k])
    wv = np.abs(o[k:, 3] - o[:-k, 3]) / (t[k:] - t[:-k])
    print(f'{os.path.basename(args.bag.rstrip("/"))}, yaw scale {args.yaw_scale}: walking speed pct 50/90/max '
          f'{np.percentile(v, [50, 90]).round(2)} / {v.max():.2f} m/s, turn rate pct 50/90 '
          f'{np.degrees(np.percentile(wv, [50, 90])).round(0)} deg/s')

    def pose(ts):
        i = int(np.clip(np.searchsorted(t, ts), 0, len(t) - 1))
        return o[i, 1], o[i, 2], o[i, 3]

    def prep(P, x, y, yaw):
        P = P[np.isfinite(P).all(1)]
        c, s = math.cos(-yaw), math.sin(-yaw)
        X = c * (P[:, 0] - x) - s * (P[:, 1] - y)
        Y = s * (P[:, 0] - x) + c * (P[:, 1] - y)
        Z = P[:, 2]
        keep = (Z > lo) & (Z < hi)
        inside = ((X > crop['min_x']) & (X < crop['max_x']) & (Y > crop['min_y']) & (Y < crop['max_y'])
                  & (Z - 0.313 > crop['min_z']) & (Z - 0.313 < crop['max_z']))
        return np.c_[X, Y][keep & ~inside]

    def measure(newest, olders, shift=0.0):
        """[(own spread, |offset| of one older message)] per wall piece and older message."""
        d = np.hypot(newest[:, 0], newest[:, 1])
        N = newest[(d > 0.5) & (d < 6.0)]
        res = []
        for c, dvec, n, t0, t1, idx in lines(N):
            own = np.median(np.abs((N[idx] - c) @ n))
            for O in olders:
                O = O + shift * n                       # self-test: push across this line
                tt = (O - c) @ dvec
                dd = (O - c) @ n
                on = (tt > t0) & (tt < t1) & (np.abs(dd) < 0.15)
                if on.sum() < 5:
                    continue
                # mode-seeking: recentre on the points within 6 cm of the current estimate,
                # so a displaced wall is followed instead of clipped by a fixed window
                m = float(np.median(dd[on]))
                for _ in range(3):
                    near = on & (np.abs(dd - m) < 0.06)
                    if near.sum() < 5:
                        break
                    m = float(np.median(dd[near]))
                if near.sum() >= 5:
                    res.append((own, abs(m)))
        return res

    idx = list(range(10, len(clouds), args.every))
    # self-test on the 0.3 s window
    st0, st5 = [], []
    for i in idx[::3]:
        x, y, yaw = pose(clouds[i][0])
        newest = prep(clouds[i][1], x, y, yaw)
        js = [j for j in range(max(0, i - 15), i) if clouds[i][0] - clouds[j][0] <= 0.3 + 1e-6]
        if not js or len(newest) < 40:
            continue
        olders = [prep(clouds[j][1], x, y, yaw) for j in js]
        st0 += measure(newest, olders)
        st5 += measure(newest, olders, shift=0.05)
    a0 = np.median([b for a, b in st0]) if st0 else float('nan')
    a5 = np.median([b for a, b in st5]) if st5 else float('nan')
    ok = len(st5) > 20 and 0.04 < a5 < 0.06
    print(f'self-test: older messages pushed 5.0 cm across each wall -> measured offset {a5 * 100:.1f} cm '
          f'(unpushed {a0 * 100:.1f} cm; {len(st5)} piece-message pairs)  {"OK" if ok else "FAIL"}')
    if not ok:
        sys.exit('self-test failed: no smear numbers')

    print(' window | front bins with a return | pairs | newest own spread | smear: |offset| of older messages, median (pct 90)')
    for W in (0.0,) + WINDOWS:
        rows, cov = [], []
        for i in idx:
            x, y, yaw = pose(clouds[i][0])
            newest = prep(clouds[i][1], x, y, yaw)
            js = [j for j in range(max(0, i - 25), i) if clouds[i][0] - clouds[j][0] <= W + 1e-6]
            olders = [prep(clouds[j][1], x, y, yaw) for j in js]
            allp = np.vstack([newest] + olders)
            dd = np.hypot(allp[:, 0], allp[:, 1])
            ok_r = (dd > 0.1) & (dd < 15)
            b = np.floor(np.degrees(np.arctan2(allp[ok_r, 1], allp[ok_r, 0])) + 180).astype(int) % 360
            bins = np.zeros(360, bool)
            bins[b] = True
            cov.append(bins[90:270].mean())
            if W > 0 and len(newest) >= 40 and olders:
                rows += measure(newest, olders)
        if W == 0:
            print(f'  {W:3.1f} s | {100 * np.mean(cov):5.1f} %                  |     - |       -           | 0 (no merge)')
            continue
        R = np.array(rows)
        print(f'  {W:3.1f} s | {100 * np.mean(cov):5.1f} %                  | {len(R):5d} | {np.median(R[:, 0]) * 100:5.1f} cm          | '
              f'{np.median(R[:, 1]) * 100:4.1f} cm ({np.percentile(R[:, 1], 90) * 100:.1f} cm)')


if __name__ == '__main__':
    main()
