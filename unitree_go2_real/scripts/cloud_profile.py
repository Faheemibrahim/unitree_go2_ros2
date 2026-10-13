#!/usr/bin/env python3
"""What does the real cloud look like around the robot? (offline, from a bag)

  ros2 run unitree_go2_real cloud_profile.py <bag_dir> [--max-clouds 400]

Takes /utlidar/cloud_deskewed (odom frame) and the robot's odometry, moves each cloud
into base_link (the robot's body frame) and base_footprint (gravity-aligned, at the body's
x, y, yaw; z unchanged), and reports for the whole bag:
  - points per cloud and their ranges;
  - the floor: the height (z in base_footprint, so z = body height above floor is
    reported separately) of the densest low layer, and the body height from the robot's
    z. The 2D scan band is set relative to these;
  - where the robot's own body and legs are: points within 0.5 m of the body origin,
    as a histogram over x, y and z in base_link; the leg ring shows as a dense blob;
  - what survives each stage of the chain (crop box in base_link, band in base_footprint,
    range) as a share of points.
Values for config/pointcloud_to_laserscan.yaml are suggested from these, not applied.
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
from sensor_msgs_py import point_cloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_check import yaw_of, stamp  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--max-clouds', type=int, default=400)
    ap.add_argument('--crop', default='-0.45,0.33,-0.22,0.22,-0.40,0.20',
                    help='min_x,max_x,min_y,max_y,min_z,max_z in base_link (the sim values)')
    ap.add_argument('--band', default='0.08,2.5', help='min_height,max_height above base_footprint (sim values)')
    args = ap.parse_args()
    crop = [float(v) for v in args.crop.split(',')]
    lo, hi = [float(v) for v in args.band.split(',')]

    meta = yaml.safe_load(open(os.path.join(args.bag, 'metadata.yaml')))['rosbag2_bagfile_information']
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=meta['storage_identifier']),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    topics = ['/utlidar/robot_odom', '/utlidar/cloud_deskewed']
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    mt = {t: get_message(types[t]) for t in topics}
    pose, n_cloud, seen = None, 0, 0
    heights, zs = [], []
    stats = dict(total=0, near=0, crop=0, band=0, rng=0)
    near_pts = []
    band_near = []          # (x, y, z_above_floor, inside_crop) of in-band points within 0.9 m
    nclouds_pts = []
    ranges = []
    every = max(1, meta['topics_with_message_count'][0]['message_count'] // 1)   # placeholder, replaced below
    cloud_total = next(t['message_count'] for t in meta['topics_with_message_count']
                       if t['topic_metadata']['name'] == '/utlidar/cloud_deskewed')
    every = max(1, cloud_total // args.max_clouds)
    while r.has_next():
        topic, data, _ = r.read_next()
        m = deserialize_message(data, mt[topic])
        if topic == topics[0]:
            p = m.pose.pose
            pose = (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation),
                    (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
            zs.append(p.position.z)
            continue
        seen += 1
        if pose is None or seen % every:
            continue
        P = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z')).astype(np.float64)
        P = P[np.isfinite(P).all(1)]
        x, y, z, yaw, (qx, qy, qz, qw) = pose
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
        Pb = (P - [x, y, z]) @ R                    # base_link
        c, s = math.cos(-yaw), math.sin(-yaw)
        Pf = np.c_[c * (P[:, 0] - x) - s * (P[:, 1] - y), s * (P[:, 0] - x) + c * (P[:, 1] - y), P[:, 2]]
        n = len(P)
        stats['total'] += n
        nclouds_pts.append(n)
        d = np.hypot(Pf[:, 0], Pf[:, 1])
        ranges.append(d)
        stats['near'] += int((np.linalg.norm(Pb, axis=1) < 0.5).sum())
        inside = ((Pb[:, 0] > crop[0]) & (Pb[:, 0] < crop[1]) & (Pb[:, 1] > crop[2]) & (Pb[:, 1] < crop[3])
                  & (Pb[:, 2] > crop[4]) & (Pb[:, 2] < crop[5]))
        stats['crop'] += int(inside.sum())
        # floor: densest layer among points in 0.4-6 m, below the body
        ring = (d > 0.4) & (d < 6.0) & (Pf[:, 2] < z)
        if ring.sum() > 200:
            h, e = np.histogram(Pf[ring, 2], bins=np.arange(Pf[ring, 2].min(), z, 0.02))
            heights.append(e[np.argmax(h)] + 0.01)
        near_pts.append(Pb[np.linalg.norm(Pb, axis=1) < 0.6])
        fl = heights[-1] if heights else 0.0
        h_above = Pf[:, 2] - fl
        sel = (h_above > lo) & (h_above < hi) & (d < 0.9)
        band_near.append(np.c_[Pb[sel, 0], Pb[sel, 1], h_above[sel], inside[sel]])
        n_cloud += 1
    floor = float(np.median(heights)) if heights else float('nan')
    print(f'{os.path.basename(args.bag.rstrip("/"))}: {n_cloud} clouds sampled, '
          f'{np.mean(nclouds_pts):.0f} points per cloud')
    print(f'body height (odometry z): median {np.median(zs):.3f} m, range {np.min(zs):.3f}..{np.max(zs):.3f}')
    print(f'floor (densest layer 0.4-6 m out, below the body) at z = {floor:.3f} m in the odom frame '
          f'-> body {np.median(zs) - floor:.3f} m above floor')
    allr = np.concatenate(ranges)
    print(f'horizontal range of points: pct 50/90/99 = {np.percentile(allr, [50, 90, 99]).round(1)} m, '
          f'max {allr.max():.1f} m')

    N = np.vstack(near_pts)
    print(f'\npoints within 0.6 m of the body origin: {len(N) / n_cloud:.0f} per cloud '
          f'({100 * len(N) / stats["total"]:.1f}% of all points)')
    if len(N):
        print('  base_link coordinates, percentiles 1 / 25 / 50 / 75 / 99:')
        for k, nm in enumerate('xyz'):
            print(f'    {nm}: {np.percentile(N[:, k], [1, 25, 50, 75, 99]).round(3)}')
        hx, ex = np.histogram(N[:, 0], bins=np.arange(-0.6, 0.65, 0.1))
        hz, ez = np.histogram(N[:, 2], bins=np.arange(-0.6, 0.45, 0.1))
        print('  x histogram (0.1 m bins from -0.6):', hx.tolist())
        print('  z histogram (0.1 m bins from -0.6):', hz.tolist())
    print(f'\ncrop box {crop} (base_link) removes {100 * stats["crop"] / stats["total"]:.1f}% of all points '
          f'= {stats["crop"] / n_cloud:.0f} per cloud, '
          f'{100 * stats["crop"] / max(stats["near"], 1):.0f}% of the points near the body')
    left = N[~((N[:, 0] > crop[0]) & (N[:, 0] < crop[1]) & (np.abs(N[:, 1]) < crop[3]) &
               (N[:, 2] > crop[4]) & (N[:, 2] < crop[5]))] if len(N) else N
    print(f'points near the body that the box does NOT remove: {len(left) / n_cloud:.1f} per cloud')
    B = np.vstack(band_near) if band_near else np.zeros((0, 4))
    print(f'\nin the band ({lo}..{hi} m above the floor) within 0.9 m of the body: {len(B) / n_cloud:.0f} per cloud')
    if len(B):
        rem = B[B[:, 3] == 0]
        print(f'  removed by the crop box: {100 * B[:, 3].mean():.0f}%;  left: {len(rem) / n_cloud:.1f} per cloud')
        if len(rem):
            print(f'  what is left, base_link x pct 1/25/50/75/99 {np.percentile(rem[:, 0], [1, 25, 50, 75, 99]).round(2)}, '
                  f'y {np.percentile(rem[:, 1], [1, 25, 50, 75, 99]).round(2)}, '
                  f'height above floor {np.percentile(rem[:, 2], [1, 50, 99]).round(2)}')
        # top-down picture in base_link of the in-band points within 0.9 m: a body part sits at
        # the same place in every cloud (a sharp blob); a room object smears
        xe, ye = np.arange(-0.6, 1.0, 0.1), np.arange(-0.9, 1.0, 0.1)
        H, _, _ = np.histogram2d(B[:, 0], B[:, 1], bins=[xe, ye])
        print('  base_link top view, points per cloud per 0.1 m cell (rows x from -0.6 forward, cols y from -0.9 left to right):')
        print('        y: ' + ' '.join(f'{v:+.1f}'[:4].rjust(4) for v in ye[:-1]))
        for i, row in enumerate(H / n_cloud):
            print(f'  x {xe[i]:+.1f}: ' + ' '.join(('  . ' if v < 0.02 else f'{v:4.1f}') for v in row))
        # where would a box have to reach to take every in-band point within 0.45 m of the body axis
        near = B[np.hypot(B[:, 0], B[:, 1]) < 0.45]
        if len(near):
            print(f'  in-band points within 0.45 m of the body centre: {len(near) / n_cloud:.1f} per cloud; '
                  f'x {np.percentile(near[:, 0], [1, 50, 99]).round(2)}, y {np.percentile(near[:, 1], [1, 50, 99]).round(2)}, '
                  f'height above floor {np.percentile(near[:, 2], [1, 50, 99]).round(2)}')


if __name__ == '__main__':
    main()
