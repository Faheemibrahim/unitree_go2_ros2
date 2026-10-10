#!/usr/bin/env python3
"""Give obstacles that are not in the map an assumed size for the global planner.

The lidar only marks the faces of an unmapped obstacle that it can see, so the
planner treats a box as a thin line and routes round its end or through the
narrow gap beside it. This node finds scan hits that lie well away from every
wall in the static map and publishes a filled halo around each one, which the
global costmap's obstacle layer marks. Gaps too narrow for the robot then read
as closed and the planner detours instead.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy import ndimage
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class UnmappedObstacleMarker(Node):
    def __init__(self):
        super().__init__('unmapped_obstacle_marker')
        # A hit closer than this to a mapped wall is treated as that wall. Larger
        # than AMCL's usual error, so walls seen through a slightly wrong pose
        # never get a halo.
        self.wall_ignore = self.declare_parameter('wall_ignore_distance', 0.45).value
        self.halo = self.declare_parameter('halo_radius', 0.30).value
        self.max_range = self.declare_parameter('max_range', 2.5).value
        self.global_frame = self.declare_parameter('global_frame', 'map').value

        self.wall_dist = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, '/unmapped_obstacles', 10)

        r = self.halo
        self.disc = np.array([(x, y) for x in np.arange(-r, r + 1e-6, 0.04)
                              for y in np.arange(-r, r + 1e-6, 0.04) if x * x + y * y <= r * r])

    def on_map(self, msg):
        info = msg.info
        grid = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
        self.res = info.resolution
        self.ox, self.oy = info.origin.position.x, info.origin.position.y
        self.wall_dist = ndimage.distance_transform_edt(grid != 100) * self.res
        self.get_logger().info(f'map received: {info.width}x{info.height} at {self.res} m')

    def on_scan(self, scan):
        if self.wall_dist is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, scan.header.frame_id,
                                                 Time.from_msg(scan.header.stamp),
                                                 timeout=Duration(seconds=0.1))
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        ok = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= self.max_range)
        r, a = ranges[ok], angles[ok] + yaw
        hx, hy = t.x + r * np.cos(a), t.y + r * np.sin(a)

        col = ((hx - self.ox) / self.res).astype(int)
        row = ((hy - self.oy) / self.res).astype(int)
        h, w = self.wall_dist.shape
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        far = np.zeros_like(inside)
        far[inside] = self.wall_dist[row[inside], col[inside]] > self.wall_ignore

        points = []
        if far.any():
            centres = np.stack([hx[far], hy[far]], axis=1)
            halo = (centres[:, None, :] + self.disc[None, :, :]).reshape(-1, 2)
            halo = np.unique(np.round(halo / 0.05) * 0.05, axis=0)
            points = [(float(x), float(y), 0.5) for x, y in halo]

        header = Header(stamp=scan.header.stamp, frame_id=self.global_frame)
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, points))


def main():
    rclpy.init()
    rclpy.spin(UnmappedObstacleMarker())


if __name__ == '__main__':
    main()
