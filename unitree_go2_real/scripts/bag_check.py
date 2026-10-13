#!/usr/bin/env python3
"""Is a recorded Go2 bag usable for measuring odometry? (offline; the robot is not needed)

  ros2 run unitree_go2_real bag_check.py <bag_dir> [<bag_dir> ...]

For each bag:
  - opens it (mcap or sqlite3, from metadata.yaml) and lists every topic with its message
    count and average rate;
  - time gaps in /utlidar/robot_odom and /utlidar/cloud_deskewed (largest gap between
    consecutive messages, by robot stamp);
  - still moments from /utlidar/robot_odom: periods of at least STILL_S seconds where the
    body moves under STILL_V m/s and turns under STILL_W rad/s; for each, its time, its
    odometry pose, and how many deskewed clouds fall inside (the lidar check needs them);
  - how much the robot moved and turned in total (odometry), and how far the last still
    is from the first one (odometry says; whether that is true is for odom_lidar_check.py).
Nothing is assumed about how the drive was planned.
"""

import math
import os
import sys

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

STILL_V = 0.02      # m/s
STILL_W = 0.02      # rad/s
STILL_S = 2.0       # s


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def check(bag):
    meta = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))['rosbag2_bagfile_information']
    dur = meta['duration']['nanoseconds'] * 1e-9
    print(f'\n=== {os.path.basename(bag.rstrip("/"))}: {meta["storage_identifier"]}, {dur:.1f} s, '
          f'{meta["message_count"]} messages')
    counts = {t['topic_metadata']['name']: t['message_count'] for t in meta['topics_with_message_count']}
    for name in sorted(counts):
        c = counts[name]
        print(f'  {name:42s} {c:7d}  {c / dur if dur else 0:7.1f} Hz' + ('   (empty)' if c == 0 else ''))

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=meta['storage_identifier']),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in ('/utlidar/robot_odom', '/utlidar/cloud_deskewed') if counts.get(t)]
    if '/utlidar/robot_odom' not in want:
        print('  no /utlidar/robot_odom data: not usable for odometry')
        return
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    msg_t = {t: get_message(types[t]) for t in want}
    odom, cloud_t = [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        m = deserialize_message(data, msg_t[topic])
        if topic == '/utlidar/robot_odom':
            p = m.pose.pose
            odom.append((stamp(m.header), p.position.x, p.position.y, yaw_of(p.orientation)))
        else:
            cloud_t.append(stamp(m.header))
    o = np.array(odom)
    o = o[np.argsort(o[:, 0])]
    t, x, y, yaw = o[:, 0], o[:, 1], o[:, 2], np.unwrap(o[:, 3])
    print(f'  robot_odom: {len(o)} msgs, largest stamp gap {np.diff(t).max() * 1000:.0f} ms')
    if cloud_t:
        ct = np.sort(np.array(cloud_t))
        print(f'  cloud_deskewed: {len(ct)} msgs, largest stamp gap {np.diff(ct).max() * 1000:.0f} ms')

    # speeds from positions over ~0.2 s, so single-message noise does not matter
    k = max(1, int(round(0.2 / np.median(np.diff(t)))))
    v = np.hypot(x[k:] - x[:-k], y[k:] - y[:-k]) / (t[k:] - t[:-k])
    w = np.abs(yaw[k:] - yaw[:-k]) / (t[k:] - t[:-k])
    still = np.r_[(v < STILL_V) & (w < STILL_W), np.zeros(k, bool)]
    runs, i = [], 0
    while i < len(still):
        if still[i]:
            j = i
            while j < len(still) and still[j]:
                j += 1
            if t[j - 1] - t[i] >= STILL_S:
                runs.append((i, j - 1))
            i = j
        else:
            i += 1
    path = np.sum(np.hypot(np.diff(x), np.diff(y)))
    turned = np.sum(np.abs(np.diff(yaw)))
    print(f'  odometry: travelled {path:.2f} m, turned {math.degrees(turned):.0f} deg in total '
          f'(net heading change {math.degrees(yaw[-1] - yaw[0]):+.0f} deg)')
    print(f'  still moments (>= {STILL_S:.0f} s, < {STILL_V} m/s, < {STILL_W} rad/s): {len(runs)}')
    ct = np.array(cloud_t) if cloud_t else np.array([])
    for n, (a, b) in enumerate(runs):
        nc = int(np.sum((ct >= t[a]) & (ct <= t[b])))
        print(f'    #{n + 1:2d} t {t[a] - t[0]:6.1f}-{t[b] - t[0]:6.1f} s ({t[b] - t[a]:4.1f} s): '
              f'odom ({x[a]:+6.2f},{y[a]:+6.2f}) yaw {math.degrees(yaw[a]):+7.1f} deg, {nc} clouds')
    if len(runs) >= 2:
        a, b = runs[0][0], runs[-1][0]
        print(f'  first -> last still, as odometry sees it: {math.hypot(x[b] - x[a], y[b] - y[a]):.2f} m apart, '
              f'heading {math.degrees(yaw[b] - yaw[a]):+.1f} deg')
    starts = runs and t[runs[0][0]] - t[0] < 10
    ends = runs and t[-1] - t[runs[-1][1]] < 10
    print(f'  still at the start: {"yes" if starts else "no"}; still at the end: {"yes" if ends else "no"}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for b in sys.argv[1:]:
        check(b)
