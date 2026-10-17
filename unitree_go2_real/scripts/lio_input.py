#!/usr/bin/env python3
"""Inputs for Point-LIO (CMU's point_lio_unilidar) on the real Go2's built-in L1.

  /utlidar/cloud_deskewed + /utlidar/robot_odom -> /go2/lio_cloud   (base_link)
  /utlidar/imu                                  -> /go2/lio_imu     (base_link, gyro only)

Robot stamps are kept (Point-LIO only needs lidar and IMU on one clock, and both come from
the robot's). Measured 2026-10-16 on our bags (offline: nav_test_tools/cmu_transform_bag.py,
imu_frame_check.py, mount_check.py):

- Cloud: the raw /utlidar/cloud does not fit Unitree's URDF mount (radar_joint) on this robot
  (68 % of points end up below the floor), so the raw cloud is not used. Unitree's own
  /utlidar/cloud_deskewed is right (flat floor at z = 0 in odom), but it sits in the robot's
  odom frame and carries ~1000 real points plus 10000 (0, 0, 0) padding points per message.
  Each message is moved back into base_link with the robot pose at its stamp (only motion
  within one message enters; the robot's long-term drift cancels), padding and the body are
  dropped, and it is written in the L1 point layout Point-LIO reads (x y z intensity ring time;
  ring 1, time 0 since it is already deskewed).
- Gyro: body <- L1 gyro is rpy (-168.26, -7.67, -60.99) deg (fit against the body gyro in
  /lowstate, 0.01 rad/s residual, same on three bags). CMU's fixed transform (flip about x,
  15.1 deg about y) is 60 deg off on this robot.
- Gyro bias (bias_mode): 'startup' (default) measures it each time this node starts: nothing is
  published until the robot has stood still for startup_still_s (judged from robot_odom's pose:
  over every 0.5 s it moved < 1 cm and turned < 0.3 deg; its twist is too noisy at rest, the yaw
  rate reads 0.01-0.03 rad/s); the mean raw gyro over that time is the bias. No motion is needed or commanded.
  If the robot does not stand still within startup_timeout_s, the fixed bias is used (warning).
  'fixed' = mean at rest over the two tuning bags; 'none' = no bias subtracted. Point-LIO also
  estimates the bias online, but starts it at zero.
- Accelerometer: in a different frame from the gyro, not resolved; published as zero, as CMU
  does (Point-LIO then runs on lidar + gyro).
"""

import bisect
import collections

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, PointCloud2, PointField

NS = 1_000_000_000
R_GYRO = Rotation.from_euler('xyz', [-168.255, -7.67, -60.985], degrees=True)
GYRO_BIAS = np.array([0.01009, 0.00047, 0.00048])      # raw frame, mean at rest, two tuning bags
FIELDS = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in
          (('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7), ('ring', 16, 4), ('time', 18, 7))]
POINT = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                  'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                  'offsets': [0, 4, 8, 12, 16, 18], 'itemsize': 22})


def stamp_ns(stamp):
    return stamp.sec * NS + stamp.nanosec


class LioInput(Node):

    def __init__(self):
        super().__init__('lio_input')
        # base_link box around the body and legs (as config/pointcloud_to_laserscan.yaml body_crop, a bit
        # longer at the back); points closer than min_range to the body origin also go.
        self.box = [self.declare_parameter(n, v).value for n, v in
                    (('min_x', -0.45), ('max_x', 0.55), ('min_y', -0.55), ('max_y', 0.50),
                     ('min_z', -0.40), ('max_z', 0.20))]
        self.max_gap_ns = int(self.declare_parameter('max_pose_gap_s', 0.02).value * NS)
        self.odom_t = collections.deque(maxlen=900)       # 6 s at 150 Hz
        self.odom_T = collections.deque(maxlen=900)       # (R, t) robot pose in odom
        self.dropped = 0
        self.bias_mode = self.declare_parameter('bias_mode', 'startup').value
        self.still_ns = int(self.declare_parameter('startup_still_s', 1.5).value * NS)
        self.timeout_ns = int(self.declare_parameter('startup_timeout_s', 20.0).value * NS)
        self.bias = {'fixed': GYRO_BIAS, 'none': np.zeros(3)}.get(self.bias_mode)   # None until measured
        if self.bias_mode == 'startup':
            self.streak = []               # (stamp ns, raw gyro) while still
            self.first_imu_ns = None
        elif self.bias is None:
            raise ValueError(f'bias_mode {self.bias_mode!r}: use startup, fixed or none')
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.cloud_pub = self.create_publisher(PointCloud2, '/go2/lio_cloud', reliable)
        self.imu_pub = self.create_publisher(Imu, '/go2/lio_imu', QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/utlidar/imu', self.on_imu, QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_odom(self, m):
        t = stamp_ns(m.header.stamp)
        if self.odom_t and t < self.odom_t[-1]:           # bag restarted
            self.odom_t.clear()
            self.odom_T.clear()
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.odom_t.append(t)
        self.odom_T.append((Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), np.array([p.x, p.y, p.z])))

    def on_cloud(self, m):
        if self.bias is None:
            return                         # Point-LIO starts once the bias is known
        t = stamp_ns(m.header.stamp)
        i = bisect.bisect_left(self.odom_t, t)
        cand = [k for k in (i - 1, i) if 0 <= k < len(self.odom_t)]
        if not cand:
            return
        k = min(cand, key=lambda k: abs(self.odom_t[k] - t))
        if abs(self.odom_t[k] - t) > self.max_gap_ns:
            self.dropped += 1
            if self.dropped in (1, 10, 100):
                self.get_logger().warn(f'{self.dropped} clouds without robot pose within {self.max_gap_ns / 1e6:.0f} ms')
            return
        R, tr = self.odom_T[k]
        off = {f.name: f.offset for f in m.fields}
        rows = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
        P = np.stack([rows[:, off[c]:off[c] + 4].copy().view(np.float32).ravel()
                      for c in ('x', 'y', 'z', 'intensity')], 1)
        P = P[np.isfinite(P).all(1) & np.any(P[:, :3] != 0, 1)]        # drop (0, 0, 0) padding
        q = (P[:, :3].astype(np.float64) - tr) @ R                       # odom -> base_link
        x0, x1, y0, y1, z0, z1 = self.box
        body = (q[:, 0] > x0) & (q[:, 0] < x1) & (q[:, 1] > y0) & (q[:, 1] < y1) & (q[:, 2] > z0) & (q[:, 2] < z1)
        q, inten = q[~body], P[~body, 3]
        rec = np.zeros(len(q), POINT)
        rec['x'], rec['y'], rec['z'], rec['intensity'], rec['ring'] = q[:, 0], q[:, 1], q[:, 2], inten, 1
        out = PointCloud2(header=m.header, height=1, width=len(q), fields=FIELDS, is_bigendian=False,
                          point_step=POINT.itemsize, row_step=POINT.itemsize * len(q), data=rec.tobytes(), is_dense=True)
        out.header.frame_id = 'base_link'
        self.cloud_pub.publish(out)

    def moving_now(self):
        """Robot pose changed over the last 0.5 s by >= 1 cm or >= 0.3 deg (or no odometry yet)."""
        if not self.odom_t or self.odom_t[-1] - self.odom_t[0] < NS // 2:
            return True
        k = bisect.bisect_left(self.odom_t, self.odom_t[-1] - NS // 2)
        (R0, t0), (R1, t1) = self.odom_T[k], self.odom_T[-1]
        turned = Rotation.from_matrix(R0.T @ R1).magnitude()
        return np.linalg.norm((t1 - t0)[:2]) >= 0.01 or turned >= np.radians(0.3)

    def measure_bias(self, t, raw):
        """Startup bias: mean raw gyro over startup_still_s of standing still (see the module docstring)."""
        if self.first_imu_ns is None:
            self.first_imu_ns = t
            self.get_logger().info(f'measuring gyro bias: keep the robot standing still for '
                                   f'{self.still_ns / NS:.1f} s')
        if self.moving_now():
            self.streak = []
        else:
            self.streak.append((t, raw))
            if t - self.streak[0][0] >= self.still_ns:
                self.bias = np.mean([g for _, g in self.streak], axis=0)
                self.get_logger().info(f'gyro bias {np.round(self.bias, 5).tolist()} rad/s (raw frame) from '
                                       f'{len(self.streak)} samples; fixed value was {GYRO_BIAS.tolist()}')
                return
        if t - self.first_imu_ns > self.timeout_ns:
            self.bias = GYRO_BIAS
            self.get_logger().warn(f'robot did not stand still for {self.still_ns / NS:.1f} s within '
                                   f'{self.timeout_ns / NS:.0f} s: using the fixed bias {GYRO_BIAS.tolist()}')

    def on_imu(self, m):
        raw = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        if self.bias is None:
            self.measure_bias(stamp_ns(m.header.stamp), raw)
            return
        w = R_GYRO.apply(raw - self.bias)
        out = Imu()
        out.header = m.header
        out.header.frame_id = 'base_link'
        out.orientation.w = 1.0
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = map(float, w)
        self.imu_pub.publish(out)          # linear_acceleration stays 0 (see the module docstring)


def main():
    rclpy.init()
    rclpy.spin(LioInput())


if __name__ == '__main__':
    main()
