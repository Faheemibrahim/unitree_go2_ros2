#!/usr/bin/env python3
"""Judge a SLAM map of a bag without ground truth.

  ros2 run unitree_go2_real map_report.py <map.yaml> <track.json> <bag_dir> [--yaw-scale 0.9814]

1. Loop closure. The true motion from the first still moment to the last one comes from the
   room: the walls at the last still are lined up with the walls at the first (the matcher of
   odom_lidar_check.py, self-tested there), applied to the corrected odometry. SLAM's own
   first-to-last motion (track.json, stills found in its odometry) is compared with that:
   position and heading difference. Pass: <= 0.15 m and <= 3 deg.
2. Walls. Straight runs of occupied cells (sequential RANSAC, contiguous, >= 1.0 m) and their
   local thickness (median over 0.5 m chunks of the band holding 90 % of the cells within
   0.25 m of the line). A doubled wall shows as a thick band. Self-tested first on a synthetic
   map with 5 cm and 15 cm walls. (A first version measured over whole 30 m lines with a
   +-0.3 m band: collinear walls and clutter saturated it at ~55 cm; rejected.)
3. Picture: <map>.png with SLAM's trail.
"""

import argparse
import json
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_check import still_moments  # noqa: E402
from odom_lidar_check import align, apply, correct, read, walls  # noqa: E402

RNG = np.random.default_rng(1)


def true_loop(bag, k):
    """(dx, dy, dyaw) of the last still in the first still's frame, from the room."""
    o, clouds = read(bag)
    o, clouds = correct(o, clouds, k)
    t, x, y, yaw = o.T
    runs = still_moments(t, x, y, yaw)
    ct = np.array([c[0] for c in clouds])

    def still_walls(run):
        i0, i1 = run
        pts = [clouds[j][1] for j in np.flatnonzero((ct >= t[i0]) & (ct <= t[i1]))]
        c = np.array([x[i0:i1 + 1].mean(), y[i0:i1 + 1].mean()])
        return walls(np.vstack(pts), c)[0], c, run[0]
    W0, c0, i0 = still_walls(runs[0])
    W1, c1, i1 = still_walls(runs[-1])
    th, sh, med, inl = align(W1, W0, c1)
    p1 = apply(np.array([[x[i1], y[i1]]]), c1, th, sh)[0]       # true end position
    yaw1 = yaw[i1] + th
    return rel((x[i0], y[i0], yaw[i0]), (p1[0], p1[1], yaw1)), med, inl


def rel(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(-a[2]), math.sin(-a[2])
    return c * dx - s * dy, s * dx + c * dy, (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi


def wall_thickness(occ_xy, res, chunk=0.5):
    """Straight wall pieces and their local thickness. Sequential RANSAC on occupied cells;
    each line's cells are split into contiguous runs (gaps <= 0.3 m), runs >= 1.0 m kept.
    Every 0.5 m chunk of a run: the cells within 0.25 m of the line, and the width of the
    band holding the central 90 % of them across the line, plus one cell. A run's thickness
    is the median over its chunks (local, so collinear separate walls and nearby clutter
    along a long line do not inflate it). Returns [(run length, thickness)]."""
    out, left = [], np.arange(len(occ_xy))
    for _ in range(12):
        if len(left) < 20:
            break
        Q = occ_xy[left]
        best = None
        for _ in range(400):
            a, b = Q[RNG.choice(len(Q), 2, replace=False)]
            d = b - a
            L = np.hypot(*d)
            if L < 0.3:
                continue
            n = np.array([-d[1], d[0]]) / L
            inl = np.abs((Q - a) @ n) < 1.5 * res
            if best is None or inl.sum() > best[1].sum():
                best = (a, inl, d / L, n)
        if best is None:
            break
        a, inl, d, n = best
        tt = (Q - a) @ d
        nn = (Q - a) @ n
        ts = np.sort(tt[inl])
        cuts = np.flatnonzero(np.diff(ts) > 0.3)
        starts, ends = np.r_[ts[0], ts[cuts + 1]], np.r_[ts[cuts], ts[-1]]
        used = np.zeros(len(Q), bool)
        for t0, t1 in zip(starts, ends):
            if t1 - t0 < 1.0:
                continue
            widths = []
            for c0 in np.arange(t0, t1 - chunk / 2, chunk):
                m = (tt >= c0) & (tt < c0 + chunk) & (np.abs(nn) < 0.25)
                if m.sum() >= 5:
                    widths.append(np.percentile(nn[m], 95) - np.percentile(nn[m], 5) + res)
            if widths:
                out.append((t1 - t0, float(np.median(widths))))
            used |= (tt >= t0) & (tt <= t1) & (np.abs(nn) < 0.25)
        used |= inl
        left = left[~used]
    return out


def wall_selftest(res=0.05):
    """Synthetic map: walls 1 cell and 3 cells thick, plus random clutter."""
    cells = []
    for y0, k in ((0.0, 1), (3.0, 3)):              # wall at y0, k cells thick, 6 m long
        for x in np.arange(0, 6, res):
            for j in range(k):
                cells.append((x, y0 + j * res))
    for x in np.arange(-2.0, 0, res):               # a perpendicular 1-cell wall
        cells.append((-0.5, x))
    cl = RNG.uniform([-1, -3], [7, 6], size=(150, 2))  # clutter
    P = np.vstack([np.array(cells), cl])
    W = wall_thickness(P, res)
    got = sorted(W, reverse=True)[:3]
    print('wall self-test (synthetic map: 6 m walls 5 cm and 15 cm thick, 2 m wall 5 cm, 150 clutter cells):')
    for L, wd in got:
        print(f'  found length {L:.1f} m, thickness {wd * 100:.0f} cm')
    th = sorted(wd for L, wd in W if L > 4)
    ok = len(th) == 2 and abs(th[0] - 0.05) <= 0.026 and abs(th[1] - 0.15) <= 0.026
    print(f'  {"OK" if ok else "FAIL"} (expected 5 and 15 cm, +-2.5 cm)')
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('track')
    ap.add_argument('bag')
    ap.add_argument('--yaw-scale', type=float, default=0.9814)
    args = ap.parse_args()

    (tdx, tdy, tdth), med, inl = true_loop(args.bag, args.yaw_scale)
    tr = [s for s in json.load(open(args.track)) if s['slam']]
    T = np.array([s['t'] for s in tr])
    O = np.array([s['odom'] for s in tr])
    S = np.array([s['slam'] for s in tr])
    runs = still_moments(T, O[:, 0], O[:, 1], np.unwrap(O[:, 2]))
    a, b = runs[0][0], runs[-1][0]
    sdx, sdy, sdth = rel(S[a], S[b])
    odx, ody, odth = rel(O[a], O[b])
    perr = math.hypot(sdx - tdx, sdy - tdy)
    herr = math.degrees(abs((sdth - tdth + math.pi) % (2 * math.pi) - math.pi))
    print(f'map {os.path.basename(args.map)}')
    print(f'loop, first still -> last still (in the first still\'s frame):')
    print(f'  from the room (walls aligned, median {med * 100:.1f} cm, {100 * inl:.0f}% matched): '
          f'({tdx:+.3f}, {tdy:+.3f}) m, {math.degrees(tdth):+.2f} deg')
    print(f'  SLAM:     ({sdx:+.3f}, {sdy:+.3f}) m, {math.degrees(sdth):+.2f} deg  -> error {perr:.3f} m, {herr:.2f} deg '
          f'{"PASS" if perr <= 0.15 and herr <= 3 else "FAIL"} (<= 0.15 m, 3 deg)')
    print(f'  odometry: ({odx:+.3f}, {ody:+.3f}) m, {math.degrees(odth):+.2f} deg  -> error '
          f'{math.hypot(odx - tdx, ody - tdy):.3f} m, '
          f'{math.degrees(abs((odth - tdth + math.pi) % (2 * math.pi) - math.pi)):.2f} deg')

    meta = yaml.safe_load(open(args.map))
    pgm = os.path.join(os.path.dirname(args.map), meta['image'])
    with open(pgm, 'rb') as f:
        f.readline()
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        img = np.frombuffer(f.read(), np.uint8).reshape(h, w)[::-1]
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]
    r, c = np.nonzero(img < 50)
    occ = np.c_[ox + (c + 0.5) * res, oy + (r + 0.5) * res]
    if not wall_selftest(res):
        sys.exit('wall self-test failed: no wall numbers')
    W = wall_thickness(occ, res)
    print(f'walls: {len(occ)} occupied cells; {len(W)} straight runs >= 1.0 m')
    for L, wd in sorted(W, reverse=True):
        print(f'  length {L:4.1f} m: thickness {wd * 100:4.0f} cm')
    if W:
        print(f'  median thickness {np.median([wd for _, wd in W]) * 100:.0f} cm (one cell = {res * 100:.0f} cm)')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 9))
        ax.imshow(img, cmap='gray', origin='lower', extent=[ox, ox + w * res, oy, oy + h * res])
        ax.plot(S[:, 0], S[:, 1], 'r-', lw=0.8, label='SLAM trail')
        ax.plot(*S[a, :2], 'go', label='first still')
        ax.plot(*S[b, :2], 'bx', label='last still')
        ax.legend(loc='upper right')
        ax.set_title(os.path.basename(args.map))
        out = os.path.splitext(args.map)[0] + '.png'
        fig.savefig(out, dpi=100, bbox_inches='tight')
        print(f'picture: {out}')
    except ImportError:
        pass


if __name__ == '__main__':
    main()
