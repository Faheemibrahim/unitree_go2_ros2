#!/usr/bin/env python3
"""Robot odometry of a bag as TUM files, raw and corrected with a yaw scale (the same
arithmetic as real_tf_bridge.py, via odom_lidar_check.correct):
  ros2 run unitree_go2_real odom_tums.py <bag_dir> [--yaw-scale 0.9814]
-> <bag>/odom_raw.tum, <bag>/odom_scaled.tum (planar: x, y, yaw; z 0)."""
import argparse
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odom_lidar_check import correct, read  # noqa: E402


def write(path, o):
    q = Rotation.from_euler('z', o[:, 3]).as_quat()
    np.savetxt(path, np.c_[o[:, 0], o[:, 1], o[:, 2], np.zeros(len(o)), q], fmt='%.6f')


ap = argparse.ArgumentParser()
ap.add_argument('bag')
ap.add_argument('--yaw-scale', type=float, default=0.9814)
a = ap.parse_args()
o, clouds = read(a.bag)
write(os.path.join(a.bag, 'odom_raw.tum'), o)
oc, _ = correct(o, clouds[:1], a.yaw_scale)
write(os.path.join(a.bag, 'odom_scaled.tum'), oc)
print(f'{len(o)} poses -> odom_raw.tum, odom_scaled.tum (yaw scale {a.yaw_scale})')
