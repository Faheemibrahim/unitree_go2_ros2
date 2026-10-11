#!/usr/bin/env python3
"""Turns Gazebo's dense render of the L1's hemisphere into what the real L1 publishes.

Gazebo's gpu_lidar can only sample a uniform grid, so lidar_4D_lidar.xacro renders
a dense one (1 deg azimuth x 1.5 deg elevation, 20 Hz) and this node picks the cells
the real sensor would sample (docs/L1_LIDAR_REALISM.md section 4):
  - each mirror sweep ("line") is 120 samples, 1.5 deg apart in pitch, along a great
    circle through the pole: azimuth psi from the horizon up to the pole, then psi+180
    back down to the opposite horizon;
  - 180 lines per second, and psi advances 360 * 11 / 180 = 22 deg per line;
  - a message holds lines_per_msg lines (12 at the Go2's ~15 Hz, about 1,440 samples;
    the Go2 publishes ~1,400, so ~3% of samples are invalid and dropped).
The pitch step equals the render's row spacing, so each sample is one rendered cell and
its point is kept exactly where the ray hit (no interpolation).

Publishes, both best effort as sensor data:
  ~/cloud (/unitree_lidar/points):   frame lidar_l1_link (the SDK frame, mounting
      face), fields x y z intensity (f32) ring (u16) time (f32) in the L1 SDK's 32-byte
      layout; ring = line index in the message, time = seconds after the stamp.
  ~/cloud_deskewed (/unitree_lidar/points_deskewed): the same points in odom, as the
      Go2's /utlidar/cloud_deskewed is in its odom frame.
Each message is delivered delay_s after its stamp (with the first render at or after
that time), as the real one arrives late from the robot's own computer. The node needs
no clock of its own: run it without use_sim_time.

Not modelled: motion during a message (each dense render is one instant, so the cloud
is already "deskewed"); per-material reflectivity.
"""

import array
import collections
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

# The L1 SDK's PCL point (unitree_lidar_sdk_pcl.h): x y z [pad] intensity ring time.
FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
    PointField(name='time', offset=24, datatype=PointField.FLOAT32, count=1),
]
POINT = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                  'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                  'offsets': [0, 4, 8, 16, 20, 24], 'itemsize': 32})


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class L1ScanPattern(Node):

    def __init__(self):
        super().__init__('l1_scan_pattern')
        p = self.declare_parameter
        self.lines_per_msg = p('lines_per_msg', 12).value     # Go2: cloud_scan_num
        self.line_rate = p('line_rate', 180.0).value          # mirror sweeps per second
        self.yaw_step = math.radians(p('yaw_step_deg', 22.0).value)    # per line
        self.yaw_drift = math.radians(p('yaw_drift_deg', 0.0).value)   # extra per line
        self.dropout = p('dropout', 0.03).value
        self.range_min = p('range_min', 0.05).value
        self.range_max = p('range_max', 20.0).value
        self.delay = p('delay_s', 0.05).value
        self.z_bias = p('z_bias', 0.0445).value       # rays start this far up the dome axis
        self.frame = p('frame_id', 'lidar_l1_link').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.max_tf_wait = p('max_tf_wait_s', 0.3).value

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.rng = np.random.default_rng(1)
        self.psi = 0.0
        self.dense_count = 0
        self.last_render = None
        self.line_debt = 0.0                    # sweeps due since the last message
        self.queue = collections.deque()
        self.pending = collections.deque()      # delivered, waiting for odom's transform
        self.pitch = np.radians(np.arange(120) * 1.5)
        self.sample_time = np.arange(120) / 43200.0    # the 43.2 kHz sampling clock

        self.create_subscription(PointCloud2, 'dense', self.on_dense, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, '~/cloud', qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(PointCloud2, '~/cloud_deskewed', qos_profile_sensor_data)

    # Everything is driven by the renders and their stamps, with no timers and no
    # sim clock: a /clock subscription (gz publishes ~1 kHz) cost this node most of a core
    # in rclpy's executor. The sweeps due since the last message accrue at line_rate;
    # once a message's worth is due, the newest render becomes the next message
    # (12 sweeps at 180/s: 15 Hz on average, on the render's 50 ms grid).
    def on_dense(self, msg):
        now = Time.from_msg(msg.header.stamp)
        if self.last_render is None:
            self.get_logger().info(
                f'dense render {msg.width} x {msg.height} in {msg.header.frame_id}; '
                f'{self.lines_per_msg} lines per message, '
                f'{self.line_rate / self.lines_per_msg:.1f} Hz, delivered {self.delay * 1000:.0f} ms late')
        else:
            self.line_debt += (now - self.last_render).nanoseconds * 1e-9 * self.line_rate
        self.last_render = now
        if self.line_debt >= self.lines_per_msg:
            self.line_debt -= self.lines_per_msg
            self.make_message(msg)
        self.deliver(now)

    def make_message(self, d):
        W, H = d.width, d.height
        names = [f.name for f in d.fields]
        off = {f.name: f.offset for f in d.fields}
        raw = np.frombuffer(d.data, np.uint8).reshape(H * W, d.point_step)   # no copy
        def col(name):
            return raw[:, off[name]:off[name] + 4].copy().view('<f4').ravel()
        xyz = np.stack([col('x'), col('y'), col('z')], axis=1)
        inten = col('intensity') if 'intensity' in names else np.zeros(H * W, np.float32)

        # The lines of this message: azimuth psi on the rising half of each sweep,
        # psi + 180 deg past the pole, elevation = pitch folded about 90 deg.
        L = self.lines_per_msg
        psi = self.psi + np.arange(L)[:, None] * (self.yaw_step + self.yaw_drift)
        self.psi = float((self.psi + L * (self.yaw_step + self.yaw_drift)) % (2 * math.pi))
        a = self.pitch[None, :]
        az = np.where(a <= math.pi / 2, psi, psi + math.pi)
        el = np.where(a <= math.pi / 2, a, math.pi - a)
        row = np.clip(np.rint(el / math.radians(1.5)).astype(int), 0, H - 1)
        colx = np.rint((np.mod(az + math.pi, 2 * math.pi)) / (2 * math.pi / W)).astype(int) % W
        idx = (row * W + colx).ravel()

        pts = xyz[idx]
        rng = np.linalg.norm(pts, axis=1)
        keep = (np.isfinite(rng) & (rng >= self.range_min) & (rng <= self.range_max)
                & (self.rng.random(len(idx)) >= self.dropout))
        line = np.repeat(np.arange(L), 120)
        t = (line / self.line_rate + np.tile(self.sample_time, L)).astype(np.float32)

        out = np.zeros(int(keep.sum()), POINT)
        out['x'], out['y'] = pts[keep, 0], pts[keep, 1]
        out['z'] = pts[keep, 2] + self.z_bias      # into the mounting-face frame
        out['intensity'] = inten[idx][keep]
        out['ring'] = line[keep]
        out['time'] = t[keep]
        self.queue.append((d.header.stamp, out))

    def cloud(self, stamp, frame, arr):
        m = PointCloud2()
        m.header.stamp, m.header.frame_id = stamp, frame
        m.height, m.width = 1, len(arr)
        m.fields, m.is_bigendian, m.point_step = FIELDS, False, 32
        m.row_step, m.is_dense = 32 * len(arr), True
        m.data = array.array('B', arr.tobytes())     # bytes would be checked per element
        return m

    def deliver(self, now):
        while self.queue and now >= Time.from_msg(self.queue[0][0]) + Duration(seconds=self.delay):
            stamp, arr = self.queue.popleft()
            self.pub.publish(self.cloud(stamp, self.frame, arr))
            self.pending.append((stamp, arr))
        # The odom copy waits for odom's transform at the stamp (the EKF can lag the sim
        # clock under load) instead of being dropped; it gives up after max_tf_wait_s.
        while self.pending:
            stamp, arr = self.pending[0]
            t = Time.from_msg(stamp)
            if not self.tf.can_transform(self.odom_frame, self.frame, t):
                if (now - t).nanoseconds * 1e-9 > self.max_tf_wait:
                    self.pending.popleft()
                    self.get_logger().warn(f'no {self.odom_frame} transform within '
                                           f'{self.max_tf_wait:.1f} s; deskewed cloud dropped',
                                           throttle_duration_sec=5.0)
                    continue
                break
            self.pending.popleft()
            tr = self.tf.lookup_transform(self.odom_frame, self.frame, t).transform
            R = quat_to_matrix(tr.rotation)
            p = np.stack([arr['x'], arr['y'], arr['z']], axis=1) @ R.T
            o = arr.copy()
            o['x'] = p[:, 0] + tr.translation.x
            o['y'] = p[:, 1] + tr.translation.y
            o['z'] = p[:, 2] + tr.translation.z
            self.pub_odom.publish(self.cloud(stamp, self.odom_frame, o))


def main():
    rclpy.init()
    rclpy.spin(L1ScanPattern())


if __name__ == '__main__':
    main()
