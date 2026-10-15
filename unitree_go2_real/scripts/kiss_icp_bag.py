#!/usr/bin/env python3
"""Lidar-only odometry for a Go2 bag with KISS-ICP (offline; no legs, no IMU).

  PYTHONPATH=~/ws/venvs/eval_pkgs:$PYTHONPATH python3 kiss_icp_bag.py <bag_dir> [--out poses.tum]
      [--max-range 15] [--voxel 0.25] [--deskew]

Feeds /utlidar/cloud (the L1's own frame, utlidar_lidar) message by message into KISS-ICP,
turns the lidar poses into body (base_link) poses with the fixed mount (radar_joint from
Unitree's go2_description), and writes them in TUM format (t x y z qx qy qz qw) next to the
robot's own odometry (<out>_odom.tum) for evo. Times are the robot's stamps.

Then prints the first-still -> last-still motion of both; score against the room with
traj_score.py. --selftest: a cloud fed twice, the second moved by a known 0.30 m / 5 deg, must
come back (one large step; KISS-ICP is built for small steps, so this test is harsh).

Measured 2026-10-16 on the Go2's L1: does NOT work out of the box. Known small steps on real
still clouds: single messages 8-75 cm / 8-14 deg off, 5 merged messages + 0.25 m voxel
2.5 cm / 0.7 deg; on lab_walk_1 it drifted 5-9 m (path 1.2-9 km for a 25 m walk). Each L1
message covers a narrow strip of directions; KISS-ICP has no IMU to bridge them. Kept for
reference; not used.
"""

import argparse
import math
import os
import sys

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_check import still_moments, stamp, yaw_of  # noqa: E402

# base_link -> utlidar_lidar (real_tf_bridge.py publishes the same)
MOUNT = np.eye(4)
MOUNT[:3, :3] = Rotation.from_euler('xyz', [0, 2.8782, 0]).as_matrix()
MOUNT[:3, 3] = [0.28945, 0, -0.046825]


def make_icp(max_range, voxel, deskew):
    from kiss_icp.config import KISSConfig
    from kiss_icp.kiss_icp import KissICP
    cfg = KISSConfig()
    cfg.data.max_range, cfg.data.min_range, cfg.data.deskew = max_range, 0.4, deskew
    cfg.mapping.voxel_size = voxel
    return KissICP(config=cfg)


def selftest(P, max_range, voxel):
    icp = make_icp(max_range, voxel, False)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('z', 5, degrees=True).as_matrix()
    T[:3, 3] = [0.30, 0.0, 0.0]
    icp.register_frame(P, np.zeros(len(P)))
    # the sensor moved by T, so the same scene is seen at T^-1 * p
    Q = (P - T[:3, 3]) @ T[:3, :3]
    icp.register_frame(Q, np.zeros(len(Q)))
    got = icp.last_pose
    e_t = np.linalg.norm(got[:3, 3] - T[:3, 3])
    e_r = math.degrees(abs(Rotation.from_matrix(got[:3, :3] @ T[:3, :3].T).magnitude()))
    ok = e_t < 0.03 and e_r < 0.3
    print(f'self-test: known move 0.30 m + 5 deg recovered within {e_t * 100:.1f} cm, {e_r:.2f} deg  '
          f'{"OK" if ok else "FAIL"}')
    return ok


def tum(path, rows):
    with open(path, 'w') as f:
        for t, T in rows:
            q = Rotation.from_matrix(T[:3, :3]).as_quat()
            f.write(f'{t:.6f} {T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f} {q[0]:.8f} {q[1]:.8f} {q[2]:.8f} {q[3]:.8f}\n')


def rel(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(-a[2]), math.sin(-a[2])
    return c * dx - s * dy, s * dx + c * dy, (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out', default=None)
    ap.add_argument('--max-range', type=float, default=15.0)
    ap.add_argument('--voxel', type=float, default=0.25)
    ap.add_argument('--deskew', action='store_true')
    ap.add_argument('--selftest', action='store_true', help='stop unless a one-step known move is recovered')
    a = ap.parse_args()
    out = a.out or os.path.join(a.bag, 'kiss_icp.tum')

    meta = yaml.safe_load(open(os.path.join(a.bag, 'metadata.yaml')))['rosbag2_bagfile_information']
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=meta['storage_identifier']),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    topics = ['/utlidar/cloud', '/utlidar/robot_odom']
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    mt = {t: get_message(types[t]) for t in topics}

    icp, lidar_rows, odom_rows, odom = make_icp(a.max_range, a.voxel, a.deskew), [], [], []
    tested = not a.selftest
    while r.has_next():
        topic, data, _ = r.read_next()
        m = deserialize_message(data, mt[topic])
        t = stamp(m.header)
        if topic == '/utlidar/robot_odom':
            p, q = m.pose.pose.position, m.pose.pose.orientation
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T[:3, 3] = [p.x, p.y, p.z]
            odom_rows.append((t, T))
            odom.append((t, p.x, p.y, yaw_of(q)))
            continue
        P = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z')).astype(np.float64)
        P = P[np.isfinite(P).all(1)]
        if not tested:
            if not selftest(P, a.max_range, a.voxel):
                sys.exit('self-test failed: no poses written')
            tested = True
        names = [f.name for f in m.fields]
        ts = (point_cloud2.read_points_numpy(m, field_names=('time',)).ravel().astype(np.float64)
              if a.deskew and 'time' in names else np.zeros(len(P)))
        icp.register_frame(P, ts)
        # lidar pose in the first lidar frame -> body pose in the first body frame
        lidar_rows.append((t, MOUNT @ icp.last_pose @ np.linalg.inv(MOUNT)))
    tum(out, lidar_rows)
    tum(os.path.splitext(out)[0] + '_odom.tum', odom_rows)
    print(f'{len(lidar_rows)} lidar poses -> {out}; robot odometry -> {os.path.splitext(out)[0]}_odom.tum')

    o = np.array(odom)
    o = o[np.argsort(o[:, 0])]
    runs = still_moments(o[:, 0], o[:, 1], o[:, 2], np.unwrap(o[:, 3]))
    t0, t1 = o[runs[0][0], 0], o[runs[-1][0], 0]
    L = np.array([(t, T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0])) for t, T in lidar_rows])
    i0, i1 = np.searchsorted(L[:, 0], [t0, t1])
    j0, j1 = runs[0][0], runs[-1][0]
    lx, ly, lth = rel(L[i0, 1:], L[min(i1, len(L) - 1), 1:])
    ox, oy, oth = rel(o[j0, 1:], o[j1, 1:])
    length = np.sum(np.hypot(np.diff(L[:, 1]), np.diff(L[:, 2])))
    print(f'first still -> last still, in the first still\'s frame:')
    print(f'  KISS-ICP: ({lx:+.3f}, {ly:+.3f}) m, {math.degrees(lth):+.2f} deg   (path {length:.1f} m)')
    print(f'  robot odometry (raw): ({ox:+.3f}, {oy:+.3f}) m, {math.degrees(oth):+.2f} deg')
    print('  compare with the room: ros2 run unitree_go2_real odom_lidar_check.py / map_report.py')


if __name__ == '__main__':
    main()
