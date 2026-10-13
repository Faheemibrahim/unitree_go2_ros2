#!/usr/bin/env python3
"""Odometry quality from a recorded odom-test bag (offline; the robot is not needed).

  ros2 run unitree_go2_real odom_check.py <bag_dir>

Nothing is assumed about how the drive was planned.

Segments come from the robot's odometry, smoothed over 0.3 s: still, walk, turn (in
place), mixed (walking while turning). Segments under 1 s are left out of the table. Reports:
  - noise and creep of /utlidar/robot_odom in every still period;
  - per walk/turn segment: odometry distance, sideways drift, yaw change, and the yaw
    change integrated independently from /utlidar/imu and /lowstate gyro z;
  - the first still pose vs the last still pose, as odometry sees it;
  - smoothness: largest step between consecutive odometry messages, stamp gaps;
  - whether /utlidar/robot_odom and /sportmodestate positions stay identical.
Writes <bag_dir>/odom_check.png (xy path, yaw over time with segments).

Stamps are the robot's own (one clock for every topic here), so no restamping is needed.
"""

import argparse
import math
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ['/utlidar/robot_odom', '/sportmodestate', '/utlidar/imu', '/lowstate']

# Segmenting thresholds, on odometry smoothed over SMOOTH_S (see segments()).
SMOOTH_S = 0.3      # s
STILL_V, STILL_W = 0.03, 0.05   # m/s, rad/s
WALK_V = 0.08       # m/s
TURN_W = 0.15       # rad/s
TURN_MAX_V = 0.12   # m/s: faster than this while turning is walking in an arc ('mixed')
MIN_SEG_S = 1.0     # s: shorter runs are absorbed only between two runs of the same kind,
                    # and are left out of the table


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    odom, sport, imu, low = [], [], [], []
    classes = {t: get_message(types[t]) for t in TOPICS if t in types}
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        t = recv_ns * 1e-9
        if topic == '/utlidar/robot_odom':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            odom.append((t, st, p.x, p.y, p.z, yaw_of(q)))
        elif topic == '/sportmodestate':
            sport.append((t, msg.velocity[0], msg.velocity[1], msg.yaw_speed,
                          msg.position[0], msg.position[1], msg.imu_state.rpy[2],
                          msg.mode, msg.gait_type, msg.error_code))
        elif topic == '/utlidar/imu':
            imu.append((t, msg.angular_velocity.z))
        elif topic == '/lowstate':
            low.append((t, msg.imu_state.gyroscope[2]))
    return (np.array(odom), np.array(sport, dtype=float), np.array(imu), np.array(low))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def segments(odom):
    """[(kind, t0, t1)] from the robot's own odometry, smoothed over SMOOTH_S so that
    single-message noise does not flip labels (the yaw speed in /sportmodestate flickers
    above any useful threshold even when walking straight). Turning in place is checked
    before walking, since the legs shuffle the body a little while turning:
      still: speed < STILL_V and turn rate < STILL_W
      turn : turn rate >= TURN_W and speed < TURN_MAX_V   (in place)
      walk : speed >= WALK_V and turn rate < TURN_W
      mixed: anything else (walking while turning, creeping)
    A run shorter than MIN_SEG_S takes the kind of its neighbours only when both
    neighbours are the same kind; different kinds are never merged."""
    t, x, y, yaw = odom[:, 0], odom[:, 2], odom[:, 3], np.unwrap(odom[:, 5])
    k = max(1, int(round(SMOOTH_S / np.median(np.diff(t)))))
    v = np.hypot(x[k:] - x[:-k], y[k:] - y[:-k]) / (t[k:] - t[:-k])
    w = np.abs(yaw[k:] - yaw[:-k]) / (t[k:] - t[:-k])
    v, w = np.r_[v, np.full(k, v[-1])], np.r_[w, np.full(k, w[-1])]
    kind = np.select([(v < STILL_V) & (w < STILL_W), (w >= TURN_W) & (v < TURN_MAX_V),
                      (v >= WALK_V) & (w < TURN_W)], ['still', 'turn', 'walk'], 'mixed')
    runs = []
    start = 0
    for i in range(1, len(kind) + 1):
        if i == len(kind) or kind[i] != kind[start]:
            runs.append([str(kind[start]), t[start], t[i - 1]])
            start = i
    for j in range(1, len(runs) - 1):
        if runs[j][2] - runs[j][1] < MIN_SEG_S and runs[j - 1][0] == runs[j + 1][0]:
            runs[j][0] = runs[j - 1][0]
    merged = []
    for r in runs:
        if merged and r[0] == merged[-1][0]:
            merged[-1][2] = r[2]
        else:
            merged.append(r)
    return [tuple(r) for r in merged]


def at(arr, t):
    return arr[np.clip(np.searchsorted(arr[:, 0], t), 0, len(arr) - 1)]


def gyro_yaw(g, t0, t1):
    m = (g[:, 0] >= t0) & (g[:, 0] <= t1)
    if m.sum() < 2:
        return float('nan')
    return math.degrees(np.trapz(g[m, 1], g[m, 0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    args = ap.parse_args()

    odom, sport, imu, low = read_bag(args.bag)
    print(f'bag {args.bag}: {len(odom)} odom, {len(sport)} sportmodestate, {len(imu)} imu, '
          f'{len(low)} lowstate msgs, {odom[-1, 0] - odom[0, 0]:.1f} s')
    modes = sorted({(int(m), int(g)) for m, g in sport[:, 7:9]})
    errors = sorted({int(e) for e in sport[:, 9]})
    print(f'(mode, gait_type) seen: {modes}   error_code seen: {errors}')

    # Smoothness.
    steps = np.hypot(np.diff(odom[:, 2]), np.diff(odom[:, 3]))
    dt = np.diff(odom[:, 1])
    print(f'odom: largest step between messages {steps.max() * 1000:.1f} mm, '
          f'largest stamp gap {dt.max() * 1000:.0f} ms, stamps backwards {int((dt < 0).sum())}')
    diffs = [math.hypot(o[2] - at(sport, o[0])[4], o[3] - at(sport, o[0])[5])
             for o in odom[::50]]
    print(f'robot_odom vs sportmodestate position: max diff {max(diffs) * 1000:.1f} mm '
          f'(sampled every 50 msgs)')

    segs = segments(odom)
    shown = [sg for sg in segs if sg[2] - sg[1] >= MIN_SEG_S]
    print(f'\nsegments: {len(segs)}, of which {len(shown)} last 1 s or more (the table shows those)')
    print('\n%-3s %-5s %7s %7s | %8s %8s %8s | %9s %9s %9s' % (
        '#', 'kind', 't0', 'dur', 'dist m', 'side m', 'odom dyaw', 'imu dyaw', 'low dyaw', 'noise mm'))
    stills = []
    for i, (kind, t0, t1) in enumerate(shown):
        a, b = at(odom, t0), at(odom, t1)
        m = (odom[:, 0] >= t0) & (odom[:, 0] <= t1)
        heading = a[5]
        dx, dy = b[2] - a[2], b[3] - a[3]
        fwd = dx * math.cos(heading) + dy * math.sin(heading)
        side = -dx * math.sin(heading) + dy * math.cos(heading)
        # Unwrapped yaw change, so a full 360 turn reads 360 and not 0.
        dyaw = math.degrees(np.sum(wrap(np.diff(odom[m, 5])))) if m.sum() > 1 else 0.0
        noise = ''
        if kind == 'still' and m.sum() > 2:
            noise = '%.1f' % (1000 * max(np.ptp(odom[m, 2]), np.ptp(odom[m, 3])))
            stills.append((t0, t1, b))
        print('%-3d %-5s %7.1f %7.1f | %8.3f %8.3f %8.1f | %9.1f %9.1f %9s' % (
            i, kind, t0 - odom[0, 0], t1 - t0, fwd, side, dyaw,
            gyro_yaw(imu, t0, t1), gyro_yaw(low, t0, t1), noise))

    # Totals over every turn-in-place segment: odometry against the two gyros, without
    # assuming how far any single turn was meant to go. (/utlidar/imu z is raw, in the
    # L1's frame, mounted upside down: its sign is opposite.)
    turn_segs = [sg for sg in shown if sg[0] == 'turn']
    # Gyro bias from the still segments, then each turn compared on its own: summing turns
    # with their signs would let left and right turns cancel.
    still_m = np.zeros(len(low), bool)
    still_i = np.zeros(len(imu), bool)
    for kind, t0, t1 in shown:
        if kind == 'still':
            still_m |= (low[:, 0] >= t0) & (low[:, 0] <= t1)
            still_i |= (imu[:, 0] >= t0) & (imu[:, 0] <= t1)
    bl = low[still_m, 1].mean() if still_m.any() else 0.0
    bi = imu[still_i, 1].mean() if still_i.any() else 0.0
    lowc, imuc = low.copy(), imu.copy()
    lowc[:, 1] -= bl
    imuc[:, 1] -= bi
    rows = []
    for _, t0, t1 in turn_segs:
        o = math.degrees(np.sum(wrap(np.diff(odom[(odom[:, 0] >= t0) & (odom[:, 0] <= t1), 5]))))
        if abs(o) > 45:
            rows.append((o, gyro_yaw(lowc, t0, t1), gyro_yaw(imuc, t0, t1)))
    print(f'\ngyro bias while still: /lowstate {math.degrees(bl):+.3f} deg/s, /utlidar/imu z {math.degrees(bi):+.3f} deg/s')
    if rows:
        R = np.array(rows)
        print(f'turns in place over 45 deg: {len(R)}; per turn, odom / gyro (bias removed):')
        for o, g, i in R:
            print(f'  odom {o:+7.1f}  lowstate {g:+7.1f} ({o / g:.4f})  L1 imu {i:+7.1f} (|ratio| {abs(o / i):.4f})')
        print(f'  median odom / lowstate gyro {np.median(R[:, 0] / R[:, 1]):.4f}; '
              f'median |odom / L1 imu| {np.median(np.abs(R[:, 0] / R[:, 2])):.4f}')
        print('  (odometry vs gyros only; how much the robot really turned: odom_lidar_check.py)')
    if len(stills) >= 2:
        s0, s1 = stills[0][2], stills[-1][2]
        print(f'first still vs last still, as odometry sees it: '
              f'{math.hypot(s1[2] - s0[2], s1[3] - s0[3]):.3f} m, '
              f'yaw {math.degrees(wrap(s1[5] - s0[5])):+.1f} deg '
              f'(over {np.sum(steps):.1f} m of odometry path)')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        ax1.plot(odom[:, 2], odom[:, 3], lw=1)
        ax1.plot(odom[0, 2], odom[0, 3], 'go', label='start')
        ax1.plot(odom[-1, 2], odom[-1, 3], 'rx', label='end')
        ax1.set_aspect('equal')
        ax1.set_xlabel('x m')
        ax1.set_ylabel('y m')
        ax1.set_title('/utlidar/robot_odom path')
        ax1.legend()
        ax1.grid(True)
        tt = odom[:, 0] - odom[0, 0]
        ax2.plot(tt, np.degrees(np.unwrap(odom[:, 5])), lw=1, label='odom yaw (unwrapped)')
        colors = {'walk': 'tab:blue', 'turn': 'tab:orange', 'mixed': 'tab:gray', 'still': 'none'}
        for kind, t0, t1 in segs:
            if kind != 'still':
                ax2.axvspan(t0 - odom[0, 0], t1 - odom[0, 0], color=colors[kind], alpha=0.15)
        ax2.set_xlabel('t s')
        ax2.set_ylabel('deg')
        ax2.set_title('yaw (blue band walk, orange band turn, grey band mixed)')
        ax2.grid(True)
        ax2.legend()
        out = os.path.join(args.bag, 'odom_check.png')
        fig.tight_layout()
        fig.savefig(out, dpi=110)
        print(f'\nplot: {out}')
    except ImportError:
        print('\n(matplotlib not installed: no plot)')


if __name__ == '__main__':
    main()
