#!/usr/bin/env python3
"""TF and PC-time stamps for the real Go2, so the sim's mapping pipeline can run on it.

The Go2 publishes no TF at all, and its clock is not synced: its stamps run ~792 s
behind the PC (measured 2026-10-13, steady to ~10 ms over a minute). Nodes that use
the PC clock (RViz, slam_toolbox's TF timeouts, Nav2 later) cannot match such stamps.

  /utlidar/robot_odom (odom -> base_link, 150 Hz)
    -> CORRECTED (see below): TF odom -> base_footprint   x, y, yaw (ground frame, as in sim)
    -> TF base_footprint -> base_link   z, roll, pitch
    -> /go2/odom        corrected, PC-time stamp
    -> /go2/odom_raw    the robot's own message, PC-time stamp
  /utlidar/cloud_deskewed (odom, 15 Hz)
    -> /go2/cloud_deskewed       re-placed into the corrected odom frame, PC-time stamp
    -> /go2/cloud_deskewed_raw   as the robot sent it, PC-time stamp
  /utlidar/cloud, /utlidar/cloud_base -> /go2/cloud, /go2/cloud_base   only if sensor_clouds
  static base_link -> utlidar_lidar   radar_joint from Unitree's go2_description

Correction (yaw_scale k; measured 2026-10-13: the robot's heading over-counts rotation by
1.84 % on odom_test_3 and 1.94 % on lab_walk_1, opposite turn directions, k = 0.9814, see
unitree_go2_real/scripts/odom_lidar_check.py). The robot integrates its own steps with its
own (too large) heading, so scaling the heading alone would leave the path bent. Instead each
message's step is taken in the BODY frame (undo the robot's heading), the heading change is
multiplied by k, and the body step is laid down along the corrected heading. z, roll and pitch
are untouched. Every deskewed cloud was placed in odom with the robot's pose at its stamp, so
it is moved by T_corrected * T_robot^-1 for that stamp (one 4x4 per cloud; points are not
re-deskewed). yaw_scale 1.0 reproduces the robot's data exactly.

Clock offset = the smallest (receive time - stamp) of robot_odom over the last
offset_window_s: the least-delayed message is the best estimate, and the window follows
slow drift between the two clocks. The cloud gets the same offset, since both stamps
come from the robot's clock. Clouds are restamped in their serialized form (the stamp
sits at a fixed offset in CDR), so the ~340 kB messages are never decoded in Python.
"""

import bisect
import collections
import functools
import math
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

NS = 1_000_000_000

# Robot cloud topic -> PC-time copy. Every one of them has std_msgs/Header first.
SENSOR_CLOUDS = {
    '/utlidar/cloud': '/go2/cloud',
    '/utlidar/cloud_base': '/go2/cloud_base',
}
HISTORY = 900       # odometry messages kept for placing clouds (6 s at 150 Hz)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def stamp_ns(stamp):
    return stamp.sec * NS + stamp.nanosec


def set_stamp(stamp, ns):
    stamp.sec, stamp.nanosec = divmod(ns, NS)


class RealTfBridge(Node):

    def __init__(self):
        super().__init__('real_tf_bridge')
        self.window_ns = int(self.declare_parameter('offset_window_s', 5.0).value * NS)
        self.k = float(self.declare_parameter('yaw_scale', 1.0).value)
        publish_raw = bool(self.declare_parameter('publish_raw', True).value)
        sensor_clouds = bool(self.declare_parameter('sensor_clouds', False).value)
        self.offset_ns = None
        self.recent = collections.deque()   # (receive ns, offset ns), offsets increasing

        # Corrected-pose state and the history clouds are placed with.
        self.prev = None                    # (robot x, y, yaw) of the previous message
        self.cor = None                     # corrected (x, y, yaw)
        self.hist_t = collections.deque(maxlen=HISTORY)    # robot stamp ns
        self.hist_T = collections.deque(maxlen=HISTORY)    # 4x4 corrected * robot^-1
        self.layout_warned = False

        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, '/go2/odom', reliable)
        self.odom_raw_pub = self.create_publisher(Odometry, '/go2/odom_raw', reliable)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.on_odom,
                                 qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, '/go2/cloud_deskewed', reliable)
        self.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', self.on_deskewed,
                                 qos_profile_sensor_data)
        if publish_raw:
            pub = self.create_publisher(PointCloud2, '/go2/cloud_deskewed_raw', reliable)
            self.create_subscription(PointCloud2, '/utlidar/cloud_deskewed',
                                     functools.partial(self.on_cloud_raw, pub),
                                     qos_profile_sensor_data, raw=True)
        if sensor_clouds:
            for src, dst in SENSOR_CLOUDS.items():
                pub = self.create_publisher(PointCloud2, dst, reliable)
                self.create_subscription(PointCloud2, src, functools.partial(self.on_cloud_raw, pub),
                                         qos_profile_sensor_data, raw=True)
        self.get_logger().info(f'yaw_scale {self.k:.4f}' + ('' if self.k != 1.0 else ' (robot data unchanged)'))
        self.publish_static()

    def publish_static(self):
        # radar_joint: xyz 0.28945 0 -0.046825, rpy 0 2.8782 0 (docs/SIM_VS_REAL.md).
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_link', 'utlidar_lidar'
        t.transform.translation.x, t.transform.translation.z = 0.28945, -0.046825
        half = 2.8782 / 2
        t.transform.rotation.y, t.transform.rotation.w = math.sin(half), math.cos(half)
        self.static_tf.sendTransform(t)

    def update_offset(self, robot_ns):
        now = self.get_clock().now().nanoseconds
        offset = now - robot_ns
        # Monotonic deque: the front is always the window's minimum.
        while self.recent and self.recent[-1][1] >= offset:
            self.recent.pop()
        self.recent.append((now, offset))
        while now - self.recent[0][0] > self.window_ns:
            self.recent.popleft()
        if self.offset_ns is None:
            self.get_logger().info(f'robot clock is {offset / NS:.3f} s behind the PC')
        self.offset_ns = self.recent[0][1]

    def on_odom(self, msg):
        robot_ns = stamp_ns(msg.header.stamp)
        self.update_offset(robot_ns)
        pc_ns = robot_ns + self.offset_ns

        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        q_yaw = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        q_tilt = quat_mul((0.0, 0.0, -q_yaw[2], q_yaw[3]), (q.x, q.y, q.z, q.w))

        # Corrected x, y, yaw (see the module docstring).
        if self.prev is None or (self.hist_t and robot_ns < self.hist_t[-1] - NS):
            self.prev, self.cor = (p.x, p.y, yaw), (p.x, p.y, yaw)
            self.hist_t.clear()
            self.hist_T.clear()
        else:
            px, py, pyaw = self.prev
            cx, cy, cyaw = self.cor
            dyaw = wrap(yaw - pyaw)
            mid = pyaw + dyaw / 2
            dx, dy = p.x - px, p.y - py
            bx = math.cos(-mid) * dx - math.sin(-mid) * dy      # step in the body frame
            by = math.sin(-mid) * dx + math.cos(-mid) * dy
            cmid = cyaw + self.k * dyaw / 2
            cx += math.cos(cmid) * bx - math.sin(cmid) * by
            cy += math.sin(cmid) * bx + math.cos(cmid) * by
            self.cor = (cx, cy, wrap(cyaw + self.k * dyaw))
            self.prev = (p.x, p.y, yaw)
        cx, cy, cyaw = self.cor
        c_yaw = (0.0, 0.0, math.sin(cyaw / 2), math.cos(cyaw / 2))
        c_quat = quat_mul(c_yaw, q_tilt)

        # T_corrected * T_robot^-1, for moving the robot's deskewed clouds.
        Rr, Rc = quat_matrix((q.x, q.y, q.z, q.w)), quat_matrix(c_quat)
        tr, tc = np.array([p.x, p.y, p.z]), np.array([cx, cy, p.z])
        M = np.eye(4)
        M[:3, :3] = Rc @ Rr.T
        M[:3, 3] = tc - M[:3, :3] @ tr
        self.hist_t.append(robot_ns)
        self.hist_T.append(M)

        raw = Odometry()
        raw.header, raw.child_frame_id, raw.pose, raw.twist = msg.header, msg.child_frame_id, msg.pose, msg.twist
        raw.header.stamp = Time(nanoseconds=pc_ns).to_msg()
        self.odom_raw_pub.publish(raw)

        out = Odometry()
        out.header, out.child_frame_id = raw.header, msg.child_frame_id
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = cx, cy, p.z
        o = out.pose.pose.orientation
        o.x, o.y, o.z, o.w = c_quat
        out.twist = msg.twist
        self.odom_pub.publish(out)

        ground = TransformStamped()
        ground.header.stamp = raw.header.stamp
        ground.header.frame_id, ground.child_frame_id = msg.header.frame_id, 'base_footprint'
        ground.transform.translation.x, ground.transform.translation.y = cx, cy
        ground.transform.rotation.z, ground.transform.rotation.w = c_yaw[2], c_yaw[3]

        body = TransformStamped()
        body.header.stamp = raw.header.stamp
        body.header.frame_id, body.child_frame_id = 'base_footprint', msg.child_frame_id
        body.transform.translation.z = p.z
        r = body.transform.rotation
        r.x, r.y, r.z, r.w = q_tilt
        self.tf.sendTransform([ground, body])

    def on_deskewed(self, msg):
        """Cloud in the robot's odom frame -> corrected odom frame, PC-time stamp."""
        if self.offset_ns is None or not self.hist_t:
            return
        robot_ns = stamp_ns(msg.header.stamp)
        i = bisect.bisect_left(self.hist_t, robot_ns)
        if i == len(self.hist_t) or (i > 0 and robot_ns - self.hist_t[i - 1] < self.hist_t[i] - robot_ns):
            i = max(i - 1, 0)
        M = self.hist_T[i]
        by_name = {f.name: f for f in msg.fields}
        ok = all(n in by_name and by_name[n].datatype == 7 and by_name[n].offset == 4 * k
                 for k, n in enumerate('xyz')) and not msg.is_bigendian
        if ok and msg.point_step >= 12:
            data = np.frombuffer(msg.data, np.uint8)
            rows = data.reshape(-1, msg.point_step)
            xyz = rows[:, :12].copy().view(np.float32).reshape(-1, 3).astype(np.float64)
            xyz = xyz @ M[:3, :3].T + M[:3, 3]
            rows[:, :12] = xyz.astype(np.float32).view(np.uint8).reshape(-1, 12)
            msg.data = bytes(data)
        elif not self.layout_warned:
            self.layout_warned = True
            self.get_logger().warn('deskewed cloud x, y, z are not float32 at 0, 4, 8: published uncorrected')
        set_stamp(msg.header.stamp, robot_ns + self.offset_ns)
        self.cloud_pub.publish(msg)

    def on_cloud_raw(self, pub, raw):
        if self.offset_ns is None:
            return                      # no odometry yet, so no offset
        # CDR: 4-byte encapsulation header (byte 1 odd = little endian), then
        # header.stamp as int32 sec, uint32 nanosec.
        fmt = '<iI' if raw[1] & 1 else '>iI'
        sec, nanosec = struct.unpack_from(fmt, raw, 4)
        sec, nanosec = divmod(sec * NS + nanosec + self.offset_ns, NS)
        out = bytearray(raw)
        struct.pack_into(fmt, out, 4, sec, nanosec)
        pub.publish(bytes(out))


def main():
    rclpy.init()
    rclpy.spin(RealTfBridge())


if __name__ == '__main__':
    main()
