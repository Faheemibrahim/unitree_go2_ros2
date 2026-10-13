#!/usr/bin/env python3
"""Is merging several lidar messages into one scan needed on the real cloud? (offline)

  ros2 run unitree_go2_real merge_sweep.py <bag_dir> [--yaw-scale 0.9814] [--every 8]

For merge windows 0, 0.1, 0.2, 0.3, 0.5 s it builds the 2D scans the chain would build from
the bag (corrected odometry, the real config's crop box and band, 1 deg bins, nearest point
per bin, range 0.1-15 m) and reports, averaged over every `--every`-th cloud:
  - bins with a return: share of the 360 one-degree bins that hold a point, and the share
    of the front half (+-90 deg), which the L1 sees best;
  - points per scan;
  - smear: how far the oldest message in the window sits from the newest one's wall points,
    median of the nearest-neighbour distances under 0.3 m (what odometry error over the
    window does to a wall; 0 for no merge).
Only the window changes; nothing else about the chain does.
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odom_lidar_check import correct, read  # noqa: E402

WINDOWS = (0.0, 0.1, 0.2, 0.3, 0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--yaw-scale', type=float, default=0.9814)
    ap.add_argument('--every', type=int, default=8)
    ap.add_argument('--config', default=None, help='default: the installed config/pointcloud_to_laserscan.yaml')
    args = ap.parse_args()
    if args.config is None:
        from ament_index_python.packages import get_package_share_directory
        args.config = os.path.join(get_package_share_directory('unitree_go2_real'),
                                   'config', 'pointcloud_to_laserscan.yaml')
    cfg = yaml.safe_load(open(args.config))
    crop, band = cfg['body_crop']['ros__parameters'], cfg['pointcloud_to_laserscan']['ros__parameters']
    lo, hi = band['min_height'], band['max_height']
    r0, r1 = band['range_min'], band['range_max']
    print(f'crop x {crop["min_x"]}..{crop["max_x"]} y {crop["min_y"]}..{crop["max_y"]}, band {lo}..{hi} m, '
          f'range {r0}..{r1} m, yaw scale {args.yaw_scale}')

    o, clouds = read(args.bag)
    o, clouds = correct(o, clouds, args.yaw_scale)
    t = o[:, 0]
    ct = np.array([c[0] for c in clouds])

    def pose(ts):
        i = int(np.clip(np.searchsorted(t, ts), 0, len(t) - 1))
        return o[i, 1], o[i, 2], o[i, 3]

    def prep(P, x, y, yaw):
        """band + crop, to the robot's ground frame; returns (n, 3) with z above floor."""
        P = P[np.isfinite(P).all(1)]
        c, s = math.cos(-yaw), math.sin(-yaw)
        X = c * (P[:, 0] - x) - s * (P[:, 1] - y)
        Y = s * (P[:, 0] - x) + c * (P[:, 1] - y)
        Z = P[:, 2]                                     # odom z origin is the floor
        keep = (Z > lo) & (Z < hi)
        # base_link z is height minus body height (0.313 m); roll and pitch ignored here
        inside = ((X > crop['min_x']) & (X < crop['max_x']) & (Y > crop['min_y']) & (Y < crop['max_y'])
                  & (Z - 0.313 > crop['min_z']) & (Z - 0.313 < crop['max_z']))
        keep &= ~inside
        return np.c_[X[keep], Y[keep], Z[keep]]

    rows = {w: [] for w in WINDOWS}
    idx = list(range(0, len(clouds), args.every))
    for i in idx:
        ts = clouds[i][0]
        x, y, yaw = pose(ts)
        for w in WINDOWS:
            js = [j for j in range(max(0, i - 8), i + 1) if ts - clouds[j][0] <= w + 1e-6]
            pts = np.vstack([prep(clouds[j][1], x, y, yaw) for j in js])
            d = np.hypot(pts[:, 0], pts[:, 1])
            ok = (d > r0) & (d < r1)
            ang = np.degrees(np.arctan2(pts[ok, 1], pts[ok, 0]))
            b = np.floor(ang + 180).astype(int) % 360
            bins = np.zeros(360, bool)
            bins[b] = True
            front = bins[90:270].mean()                 # bins for -90..+90 deg
            smear = np.nan
            if len(js) > 1:
                a = prep(clouds[js[0]][1], x, y, yaw)[:, :2]
                n_ = prep(clouds[js[-1]][1], x, y, yaw)[:, :2]
                if len(a) > 50 and len(n_) > 50:
                    dd, _ = cKDTree(n_).query(a)
                    dd = dd[dd < 0.3]
                    smear = float(np.median(dd)) if len(dd) > 20 else np.nan
            rows[w].append((bins.mean(), front, int(ok.sum()), smear))
    print(f'\n{len(idx)} scans per window ({os.path.basename(args.bag.rstrip("/"))})')
    print(' window | bins with a return | front half | points per scan | smear (oldest vs newest, median)')
    for w in WINDOWS:
        a = np.array(rows[w], float)
        sm = np.nanmedian(a[:, 3]) if np.isfinite(a[:, 3]).any() else float('nan')
        print(f'  {w:3.1f} s |  {100 * a[:, 0].mean():5.1f} %           | {100 * a[:, 1].mean():5.1f} %    | '
              f'{a[:, 2].mean():8.0f}        | ' + ('-' if w == 0 else f'{sm * 100:.1f} cm'))


if __name__ == '__main__':
    main()
