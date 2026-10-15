#!/usr/bin/env python3
"""Write an Odometry topic to a TUM file (t x y z qx qy qz qw, message stamps) while a bag
plays. Stops after --quiet-s seconds without messages.
  ros2 run unitree_go2_real odom_to_tum.py <topic> <out.tum> [--quiet-s 10]"""
import argparse
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('topic')
    ap.add_argument('out')
    ap.add_argument('--quiet-s', type=float, default=10.0)
    a = ap.parse_args()
    rclpy.init()
    n = Node('odom_to_tum')
    rows, last = [], [time.monotonic()]

    def on(m):
        last[0] = time.monotonic()
        p, q = m.pose.pose.position, m.pose.pose.orientation
        rows.append(f'{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
                    f'{q.x:.8f} {q.y:.8f} {q.z:.8f} {q.w:.8f}')
    n.create_subscription(Odometry, a.topic, on, QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
    while not rows or time.monotonic() - last[0] < a.quiet_s:
        rclpy.spin_once(n, timeout_sec=0.1)
    open(a.out, 'w').write('\n'.join(rows) + '\n')
    print(f'{len(rows)} poses -> {a.out}')


if __name__ == '__main__':
    main()
