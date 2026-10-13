#!/usr/bin/env python3
"""Record where SLAM and odometry put the robot while a bag plays through real_mapping.launch.py.

  ros2 run unitree_go2_real slam_track.py <out.json> [--quiet-s 8]

At 5 Hz (by message stamp): the odometry pose from /go2/odom and SLAM's pose (TF map ->
base_footprint, latest). Stops when /go2/odom has been silent for --quiet-s seconds (the
bag ended) and writes [{t, odom: [x, y, yaw], slam: [x, y, yaw] or null}, ...].
"""

import argparse
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--quiet-s', type=float, default=8.0)
    args = ap.parse_args()
    rclpy.init()
    n = Node('slam_track')
    buf = Buffer()
    TransformListener(buf, n)
    track, last = [], [None, time.monotonic()]

    def on_odom(m):
        last[1] = time.monotonic()
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if last[0] is not None and t - last[0] < 0.2:
            return
        last[0] = t
        p = m.pose.pose
        try:
            s = buf.lookup_transform('map', 'base_footprint', Time()).transform
            sl = [s.translation.x, s.translation.y, yaw_of(s.rotation)]
        except Exception:   # noqa: BLE001 - no map yet
            sl = None
        track.append({'t': t, 'odom': [p.position.x, p.position.y, yaw_of(p.orientation)], 'slam': sl})

    n.create_subscription(Odometry, '/go2/odom', on_odom, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    while not track or time.monotonic() - last[1] < args.quiet_s:
        rclpy.spin_once(n, timeout_sec=0.1)
    json.dump(track, open(args.out, 'w'))
    print(f'{len(track)} samples ({sum(s["slam"] is not None for s in track)} with a SLAM pose) -> {args.out}')


if __name__ == '__main__':
    main()
