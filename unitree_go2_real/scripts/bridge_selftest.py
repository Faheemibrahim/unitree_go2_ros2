#!/usr/bin/env python3
"""Does real_tf_bridge.py do what its docstring says? (run while a bag plays through it)

  terminal A: ros2 run unitree_go2_real real_tf_bridge.py --ros-args --params-file <real_bridge.yaml>
  terminal B: ros2 bag play ~/ws/bags/odom_test_3 --start-offset 15
  terminal C: ros2 run unitree_go2_real bridge_selftest.py [--seconds 40] [--yaw-scale 0.9814]

Listens to /go2/odom_raw, /go2/odom, /go2/cloud_deskewed_raw, /go2/cloud_deskewed and
checks, with its own implementation of the arithmetic:
  1. /go2/odom == the robot's odometry corrected with yaw_scale (position, yaw);
  2. z and the tilt (roll, pitch) are unchanged;
  3. each corrected cloud == the raw cloud moved by T_corrected * T_raw^-1 at its stamp;
  4. raw and corrected clouds have the same stamp and point count.
Prints the largest deviations; exit status 1 if any is above its limit.
"""

import argparse
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rpy_tilt(q):
    """roll and pitch with the yaw taken out."""
    y = yaw_of(q)
    c, s = math.cos(-y / 2), math.sin(-y / 2)
    # q_tilt = q_yaw^-1 * q
    zq = (0.0, 0.0, -math.sin(y / 2), math.cos(y / 2))
    ax, ay, az, aw = zq
    bx, by, bz, bw = q.x, q.y, q.z, q.w
    t = (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
         aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)
    return 2 * t[0], 2 * t[1]


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=40.0)
    ap.add_argument('--yaw-scale', type=float, default=0.9814)
    args = ap.parse_args()
    rclpy.init()
    n = Node('bridge_selftest')
    q = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
    raw, cor, craw, ccor = {}, {}, {}, {}
    n.create_subscription(Odometry, '/go2/odom_raw', lambda m: raw.__setitem__(stamp_ns(m.header), m), q)
    n.create_subscription(Odometry, '/go2/odom', lambda m: cor.__setitem__(stamp_ns(m.header), m), q)
    n.create_subscription(PointCloud2, '/go2/cloud_deskewed_raw', lambda m: craw.__setitem__(stamp_ns(m.header), m), q)
    n.create_subscription(PointCloud2, '/go2/cloud_deskewed', lambda m: ccor.__setitem__(stamp_ns(m.header), m), q)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    print(f'received: odom_raw {len(raw)}, odom {len(cor)}, cloud_raw {len(craw)}, cloud {len(ccor)}')
    ok = True

    # 1-2. odometry against our own implementation (needs consecutive messages)
    ts = sorted(set(raw) & set(cor))
    if len(ts) < 100:
        raise SystemExit('too few odometry messages: is the bag playing through the bridge?')
    k = args.yaw_scale
    px = py = pyaw = None
    d_pos = d_yaw = d_z = d_tilt = 0.0
    for t in ts:
        r, c = raw[t].pose.pose, cor[t].pose.pose
        yaw = yaw_of(r.orientation)
        if px is None:
            cx, cy, cyaw = r.position.x, r.position.y, yaw
        else:
            dyaw = wrap(yaw - pyaw)
            mid = pyaw + dyaw / 2
            dx, dy = r.position.x - px, r.position.y - py
            bx = math.cos(-mid) * dx - math.sin(-mid) * dy
            by = math.sin(-mid) * dx + math.cos(-mid) * dy
            cmid = cyaw + k * dyaw / 2
            cx += math.cos(cmid) * bx - math.sin(cmid) * by
            cy += math.sin(cmid) * bx + math.cos(cmid) * by
            cyaw = wrap(cyaw + k * dyaw)
        px, py, pyaw = r.position.x, r.position.y, yaw
        if t == ts[0]:
            # the bridge started on an earlier message; anchor on its output, then follow
            cx, cy, cyaw = c.position.x, c.position.y, yaw_of(c.orientation)
            continue
        d_pos = max(d_pos, math.hypot(cx - c.position.x, cy - c.position.y))
        d_yaw = max(d_yaw, abs(wrap(cyaw - yaw_of(c.orientation))))
        d_z = max(d_z, abs(r.position.z - c.position.z))
        a, b = rpy_tilt(r.orientation), rpy_tilt(c.orientation)
        d_tilt = max(d_tilt, abs(a[0] - b[0]), abs(a[1] - b[1]))
    # corrected minus the robot's own, directly (0 when yaw_scale is 1.0)
    dx = max(math.hypot(cor[t].pose.pose.position.x - raw[t].pose.pose.position.x,
                        cor[t].pose.pose.position.y - raw[t].pose.pose.position.y) for t in ts)
    dh = max(abs(wrap(yaw_of(cor[t].pose.pose.orientation) - yaw_of(raw[t].pose.pose.orientation))) for t in ts)
    print(f'corrected minus the robot\'s own: largest position difference {dx * 1000:.1f} mm, '
          f'heading {math.degrees(dh):.2f} deg' + ('   (yaw_scale 1.0: both must be ~0)' if args.yaw_scale == 1.0 else ''))
    if args.yaw_scale == 1.0:
        ok &= dx < 1e-6 and dh < math.radians(0.001)
    total_yaw = abs(wrap(yaw_of(raw[ts[-1]].pose.pose.orientation) - yaw_of(raw[ts[0]].pose.pose.orientation)))
    print(f'odometry over {len(ts)} messages: max position difference to the independent '
          f'implementation {d_pos * 1000:.3f} mm, yaw {math.degrees(d_yaw):.4f} deg; '
          f'z {d_z * 1000:.4f} mm, tilt {math.degrees(d_tilt):.4f} deg')
    ok &= d_pos < 1e-3 and d_yaw < math.radians(0.01) and d_z < 1e-6 and d_tilt < math.radians(0.01)

    # 3-4. clouds: corrected = raw moved by T_corrected * T_raw^-1 (planar yaw + shift; tilt cancels)
    worst, checked, count_mismatch = 0.0, 0, 0
    for t in sorted(set(craw) & set(ccor)):
        # odometry stamp nearest the cloud's stamp (cloud stamps are robot stamps + the same offset)
        j = min(ts, key=lambda s: abs(s - t))
        r, c = raw[j].pose.pose, cor[j].pose.pose
        d = yaw_of(c.orientation) - yaw_of(r.orientation)
        a = np.frombuffer(craw[t].data, np.uint8).reshape(-1, craw[t].point_step)
        b = np.frombuffer(ccor[t].data, np.uint8).reshape(-1, ccor[t].point_step)
        if a.shape != b.shape:
            count_mismatch += 1
            continue
        pa = a[:, :12].copy().view(np.float32).reshape(-1, 3).astype(np.float64)
        pb = b[:, :12].copy().view(np.float32).reshape(-1, 3).astype(np.float64)
        good = np.isfinite(pa).all(1)
        R = np.array([[math.cos(d), -math.sin(d)], [math.sin(d), math.cos(d)]])
        exp = (pa[good, :2] - [r.position.x, r.position.y]) @ R.T + [c.position.x, c.position.y]
        worst = max(worst, float(np.abs(exp - pb[good, :2]).max()), float(np.abs(pa[good, 2] - pb[good, 2]).max()))
        checked += 1
    print(f'clouds: {checked} compared, {count_mismatch} with different sizes; '
          f'largest point difference to the expected move {worst * 1000:.2f} mm')
    ok &= checked > 5 and count_mismatch == 0 and worst < 0.02      # nearest odometry message: up to one step apart
    print('turning in these messages: raw heading change '
          f'{math.degrees(total_yaw):.1f} deg (net)')
    print('PASS' if ok else 'FAIL')
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
