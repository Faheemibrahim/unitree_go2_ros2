#!/usr/bin/env python3
"""While a bag plays through real_mapping.launch.py: SLAM's pose (TF map -> base_footprint,
latest) written as TUM against the ROBOT's stamps (from /utlidar/robot_odom), so it scores
with traj_score.py exactly like any odometry. 10 Hz. Stops after --quiet-s without odometry.
  ros2 run unitree_go2_real slam_tum.py <out.tum> [--quiet-s 8]"""
import argparse
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--quiet-s', type=float, default=8.0)
    a = ap.parse_args()
    rclpy.init()
    n = Node('slam_tum')
    buf = Buffer()
    TransformListener(buf, n)
    rows, last = [], [None, time.monotonic()]

    def on(m):
        last[1] = time.monotonic()
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if last[0] is not None and t - last[0] < 0.1:
            return
        try:
            s = buf.lookup_transform('map', 'base_footprint', Time()).transform
        except Exception:   # noqa: BLE001 - no map yet
            return
        last[0] = t
        p, q = s.translation, s.rotation
        rows.append(f'{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.8f} {q.y:.8f} {q.z:.8f} {q.w:.8f}')
    n.create_subscription(Odometry, '/utlidar/robot_odom', on, qos_profile_sensor_data)
    while not rows or time.monotonic() - last[1] < a.quiet_s:
        rclpy.spin_once(n, timeout_sec=0.05)
    open(a.out, 'w').write('\n'.join(rows) + '\n')
    print(f'{len(rows)} SLAM poses -> {a.out}')


if __name__ == '__main__':
    main()
