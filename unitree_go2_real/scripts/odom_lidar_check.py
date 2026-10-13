#!/usr/bin/env python3
"""How wrong is the Go2's odometry? Measured from the room itself (offline, from a bag).

  ros2 run unitree_go2_real odom_lidar_check.py <bag_dir> [--selftest-only]

Idea: every time the robot stands still, the lidar sees the room. /utlidar/cloud_deskewed is
already placed in the odom frame using the robot's own odometry. If odometry were perfect,
the walls seen at every still moment would land exactly on top of the walls seen at the
first one. So for each later still moment the walls are lined up with the first still's
walls (2D ICP: a rotation about the robot plus a shift). That rotation is the odometry's
heading error at that moment, and the shift its position error, measured against the room,
not against how far the drive was meant to turn.

Self-test first: two halves of the first still's clouds, one rotated and shifted by known
amounts; the matcher must recover them. If it does not, no odometry number is printed.

For every still moment: the heading change odometry claims since the first still (signed,
and total turning in between), the correction the walls need, the position error, and the
match quality (median distance between matched wall points, share of points matched within
5 cm). Then a fit: heading error as a fraction of the heading change (if odometry over-
counts rotation by e, the correction is -e x the change). Writes <bag_dir>/odom_lidar_check.png:
walls of every still placed by odometry (left) and after the correction (right).
"""

import math
import os
import sys

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy import ndimage
from scipy.spatial import cKDTree
from sensor_msgs_py import point_cloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_check import still_moments, yaw_of, stamp  # noqa: E402

NEAR, FAR = 0.6, 8.0          # m from the robot: drop its own body / sparse far returns
WALL_LO, WALL_HI = 0.15, 1.8  # m above the floor: walls and furniture, not the floor
VOXEL = 0.05
COARSE = np.radians(np.arange(-30, 30.25, 0.5))   # heading search, deg
SHIFT_MAX = 1.2                                    # m, shift search either way
GRID = 0.05                                        # m, search raster
MATCH_OK = 0.05               # m: a point counts as matched within this


def read(bag):
    meta = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))['rosbag2_bagfile_information']
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=meta['storage_identifier']),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    topics = ['/utlidar/robot_odom', '/utlidar/cloud_deskewed']
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    mt = {t: get_message(types[t]) for t in topics}
    odom, clouds = [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        m = deserialize_message(data, mt[topic])
        if topic == topics[0]:
            p = m.pose.pose
            odom.append((stamp(m.header), p.position.x, p.position.y, yaw_of(p.orientation)))
        else:
            if m.header.frame_id != 'odom':
                sys.exit(f'cloud_deskewed is in {m.header.frame_id!r}, expected odom')
            clouds.append((stamp(m.header), point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'))))
    o = np.array(odom)
    o = o[np.argsort(o[:, 0])]
    o[:, 3] = np.unwrap(o[:, 3])
    return o, clouds


def walls(points, centre):
    """Wall points (2D) around the robot: floor found as the densest low height layer."""
    P = points[np.isfinite(points).all(1)]
    d = np.hypot(P[:, 0] - centre[0], P[:, 1] - centre[1])
    P = P[(d > NEAR) & (d < FAR)]
    z = P[:, 2]
    low = z[z < np.median(z)]
    h, e = np.histogram(low, bins=np.arange(low.min(), low.max() + 0.05, 0.05))
    floor = e[np.argmax(h)] + 0.025
    P = P[(z > floor + WALL_LO) & (z < floor + WALL_HI)]
    k = np.floor(P[:, :2] / VOXEL).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return P[idx, :2], floor


def rot(th):
    return np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])


def apply(S, c, th, t):
    return (S - c) @ rot(th).T + c + t


def icp(src, tree, c, th0, iters, gates, t0=None):
    """Rotation th about c plus shift t that moves src onto the tree's points."""
    th, t = th0, (np.zeros(2) if t0 is None else np.array(t0, float))
    for it in range(iters):
        S = apply(src, c, th, t)
        d, j = tree.query(S, distance_upper_bound=gates[min(it, len(gates) - 1)])
        ok = np.isfinite(d)
        if ok.sum() < 20:
            break
        A, B = S[ok], tree.data[j[ok]]
        ma, mb = A.mean(0), B.mean(0)
        H = (A - ma).T @ (B - mb)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1] *= -1
            R = Vt.T @ U.T
        dth = math.atan2(R[1, 0], R[0, 0])
        # A -> R A + (mb - R ma); written as a rotation about c plus a shift
        tr = mb - R @ ma
        newc = R @ (c + t) + tr
        th += dth
        t = newc - c
    d, _ = tree.query(apply(src, c, th, t))
    return th, t, float(np.median(d)), float(np.mean(d < MATCH_OK))


def align(src, dst, c):
    """Exhaustive coarse search, then ICP. Coarse: the reference walls become a likelihood
    raster (1 on a wall, falling off over ~5 cm); for every heading (0.5 deg steps over +-30)
    the rotated source is rasterised and correlated with it by FFT, which scores every
    shift up to +-1.2 m at once. The best heading and shift seed the ICP refinement.
    Returns th, t, median distance, share matched."""
    lo = np.minimum(src.min(0), dst.min(0)) - SHIFT_MAX - 1.0
    hi = np.maximum(src.max(0), dst.max(0)) + SHIFT_MAX + 1.0
    n = np.ceil((hi - lo) / GRID).astype(int)
    occ = np.zeros((n[1], n[0]), bool)
    ij = ((dst - lo) / GRID).astype(int)
    occ[ij[:, 1], ij[:, 0]] = True
    like = np.exp(-(ndimage.distance_transform_edt(~occ) * GRID) ** 2 / (2 * 0.05 ** 2))
    F_like = np.fft.rfft2(like)
    m = int(round(SHIFT_MAX / GRID))
    best = (-1.0, 0.0, np.zeros(2))
    for th in COARSE:
        S = apply(src, c, th, np.zeros(2))
        ij = ((S - lo) / GRID).astype(int)
        img = np.zeros_like(like)
        img[np.clip(ij[:, 1], 0, n[1] - 1), np.clip(ij[:, 0], 0, n[0] - 1)] = 1.0
        corr = np.fft.irfft2(F_like * np.conj(np.fft.rfft2(img)), s=like.shape)
        # corr[dy, dx] = sum like[p + (dy, dx)] * img[p]; keep shifts within +-m cells
        win = np.concatenate([corr[:m + 1], corr[-m:]], 0)
        win = np.concatenate([win[:, :m + 1], win[:, -m:]], 1)
        k = np.unravel_index(np.argmax(win), win.shape)
        dy = k[0] if k[0] <= m else k[0] - win.shape[0]
        dx = k[1] if k[1] <= m else k[1] - win.shape[1]
        score = win[k] / img.sum()
        if score > best[0]:
            best = (score, th, np.array([dx, dy]) * GRID)
    tree = cKDTree(dst)
    return icp(src, tree, c, best[1], 40, [0.3] * 5 + [0.15] * 10 + [0.08], t0=best[2])


def selftest(clouds, o, run):
    a, b = o[run[0], 0], o[run[1], 0]
    inside = [p for t, p in clouds if a <= t <= b]
    c = o[run[0], 1:3]
    A, _ = walls(np.vstack(inside[0::2]), c)
    B, _ = walls(np.vstack(inside[1::2]), c)
    ok = True
    print('self-test (halves of the first still; known rotation and shift must come back):')
    for deg, shift in ((7.0, (0.30, -0.15)), (-20.0, (-0.5, 0.4)), (0.0, (0.0, 0.0))):
        th = math.radians(deg)
        moved = apply(B, c, th, np.array(shift))
        rth, rt, med, inl = align(moved, A, c)
        # moved = rot(th)(B - c) + c + shift; the matcher returns the inverse
        e_deg = math.degrees(rth) + deg
        back = apply(apply(np.array([c]), c, th, np.array(shift)), c, rth, rt)[0]
        e_pos = float(np.hypot(*(back - c)))
        good = abs(e_deg) < 0.3 and e_pos < 0.03
        ok &= good
        print(f'  applied {deg:+6.1f} deg, shift ({shift[0]:+.2f},{shift[1]:+.2f}) m -> recovered with error '
              f'{e_deg:+.3f} deg, {e_pos * 100:.1f} cm; median {med * 100:.1f} cm, matched {100 * inl:.0f}%  '
              f'{"OK" if good else "FAIL"}')
    return ok


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    bag = sys.argv[1]
    o, clouds = read(bag)
    t, x, y, yaw = o.T
    runs = still_moments(t, x, y, yaw)
    print(f'{os.path.basename(bag.rstrip("/"))}: {len(clouds)} clouds, {len(runs)} still moments')
    if len(runs) < 2:
        sys.exit('need at least two still moments')
    if not selftest(clouds, o, runs[0]):
        sys.exit('self-test failed: the matcher cannot be trusted on this bag; no odometry numbers.')
    if '--selftest-only' in sys.argv:
        return

    ct = np.array([c[0] for c in clouds])
    def still_walls(run):
        i0, i1 = run
        pts = [clouds[k][1] for k in np.flatnonzero((ct >= t[i0]) & (ct <= t[i1]))]
        c = np.array([x[i0:i1 + 1].mean(), y[i0:i1 + 1].mean()])
        W, floor = walls(np.vstack(pts), c)
        return W, c, len(pts), floor
    ref, c0, n0, _ = still_walls(runs[0])
    print(f'\nreference = still #1 (t {t[runs[0][0]] - t[0]:.1f} s, {n0} clouds, {len(ref)} wall points)')
    print(' still |  t (s) | odom heading change | total turned | correction needed | position | match quality')
    print('       |        | since still #1 (deg) | between (deg) |  heading (deg)    | error (m)| median / matched')
    rows, placed = [], [(ref, None)]
    absturn = np.r_[0, np.cumsum(np.abs(np.diff(yaw)))]
    for n, run in enumerate(runs[1:], start=2):
        W, c, nc, _ = still_walls(run)
        th, sh, med, inl = align(W, ref, c)
        dyaw = math.degrees(yaw[run[0]] - yaw[runs[0][0]])
        turned = math.degrees(absturn[run[0]] - absturn[runs[0][0]])
        perr = float(np.hypot(*sh))
        trusted = med < 0.05 and inl > 0.5
        rows.append((dyaw, turned, math.degrees(th), perr, trusted))
        placed.append((W, (c, th, sh)))
        print(f'  #{n:2d}  | {t[run[0]] - t[0]:6.1f} | {dyaw:+18.1f} | {turned:12.0f} | {math.degrees(th):+16.2f}  | '
              f'{perr:7.3f}  | {med * 100:4.1f} cm / {100 * inl:3.0f}%{"" if trusted else "  (weak match, not used)"}')

    R = np.array([r for r in rows if r[4]])
    if len(R):
        # correction = -e * change  ->  e = -sum(c*x)/sum(x*x), for x = signed change and x = total turned
        for col, name in ((0, 'signed heading change'), (1, 'total turning')):
            xs, cs = R[:, col], R[:, 2]
            e = -np.sum(cs * xs) / np.sum(xs * xs)
            rms = math.sqrt(np.mean((cs + e * xs) ** 2))
            print(f'fit against {name:22s}: odometry over-counts rotation by {100 * e:+.2f}% '
                  f'(yaw scale {1 / (1 + e):.4f}); residual {rms:.2f} deg rms over {len(R)} stills')
    else:
        print('no trusted matches: no fit')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(14, 7), sharex=True, sharey=True)
        cmap = plt.get_cmap('tab10')
        for k, (W, corr) in enumerate(placed):
            ax[0].scatter(W[:, 0], W[:, 1], s=1, color=cmap(k % 10), label=f'still #{k + 1}')
            Wc = W if corr is None else apply(W, corr[0], corr[1], corr[2])
            ax[1].scatter(Wc[:, 0], Wc[:, 1], s=1, color=cmap(k % 10))
        ax[0].plot(x, y, 'k-', lw=0.5)
        ax[0].set_title('walls placed by odometry (should coincide)')
        ax[1].set_title('after the measured correction')
        for a in ax:
            a.set_aspect('equal')
            a.grid(alpha=0.3)
        ax[0].legend(markerscale=8, fontsize=7, loc='upper right')
        out = os.path.join(bag, 'odom_lidar_check.png')
        fig.savefig(out, dpi=90, bbox_inches='tight')
        print(f'picture: {out}')
    except ImportError:
        pass


if __name__ == '__main__':
    main()
