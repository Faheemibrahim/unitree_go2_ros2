#!/usr/bin/env python3
"""TF and PC-time stamps for the real Go2, so the sim's mapping pipeline can run on it.

The Go2 publishes no TF at all, and its clock is not synced: its stamps run ~792 s
behind the PC (measured 2026-10-13, steady to ~10 ms over a minute). Nodes that use
the PC clock (RViz, slam_toolbox's TF timeouts, Nav2 later) cannot match such stamps.

  /utlidar/robot_odom (odom -> base_link, 150 Hz)
    -> TF odom -> base_footprint   x, y, yaw (gravity-aligned ground frame, as in sim)
    -> TF base_footprint -> base_link   z, roll, pitch
    -> /go2/odom   same message, PC-time stamp
  /utlidar/cloud_deskewed (odom, 15 Hz) -> /go2/cloud_deskewed   PC-time stamp
  static base_link -> utlidar_lidar   radar_joint from Unitree's go2_description

Clock offset = the smallest (receive time - stamp) of robot_odom over the last
offset_window_s: the least-delayed message is the best estimate, and the window follows
slow drift between the two clocks. The cloud gets the same offset, since both stamps
come from the robot's clock. Clouds are restamped in their serialized form (the stamp
sits at a fixed offset in CDR), so the ~340 kB messages are never decoded in Python.
"""

import collections
import math
import struct

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

NS = 1_000_000_000


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def stamp_ns(stamp):
    return stamp.sec * NS + stamp.nanosec


def set_stamp(stamp, ns):
    stamp.sec, stamp.nanosec = divmod(ns, NS)


class RealTfBridge(Node):

    def __init__(self):
        super().__init__('real_tf_bridge')
        self.window_ns = int(self.declare_parameter('offset_window_s', 5.0).value * NS)
        self.offset_ns = None
        self.recent = collections.deque()   # (receive ns, offset ns), offsets increasing

        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, '/go2/odom', reliable)
        self.cloud_pub = self.create_publisher(PointCloud2, '/go2/cloud_deskewed', reliable)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', self.on_cloud,
                                 qos_profile_sensor_data, raw=True)
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
        self.update_offset(stamp_ns(msg.header.stamp))
        set_stamp(msg.header.stamp, stamp_ns(msg.header.stamp) + self.offset_ns)
        self.odom_pub.publish(msg)

        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        q_yaw = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        q_tilt = quat_mul((0.0, 0.0, -q_yaw[2], q_yaw[3]), (q.x, q.y, q.z, q.w))

        ground = TransformStamped()
        ground.header.stamp = msg.header.stamp
        ground.header.frame_id, ground.child_frame_id = msg.header.frame_id, 'base_footprint'
        ground.transform.translation.x, ground.transform.translation.y = p.x, p.y
        ground.transform.rotation.z, ground.transform.rotation.w = q_yaw[2], q_yaw[3]

        body = TransformStamped()
        body.header.stamp = msg.header.stamp
        body.header.frame_id, body.child_frame_id = 'base_footprint', msg.child_frame_id
        body.transform.translation.z = p.z
        r = body.transform.rotation
        r.x, r.y, r.z, r.w = q_tilt
        self.tf.sendTransform([ground, body])

    def on_cloud(self, raw):
        if self.offset_ns is None:
            return                      # no odometry yet, so no offset
        # CDR: 4-byte encapsulation header (byte 1 odd = little endian), then
        # header.stamp as int32 sec, uint32 nanosec.
        fmt = '<iI' if raw[1] & 1 else '>iI'
        sec, nanosec = struct.unpack_from(fmt, raw, 4)
        sec, nanosec = divmod(sec * NS + nanosec + self.offset_ns, NS)
        out = bytearray(raw)
        struct.pack_into(fmt, out, 4, sec, nanosec)
        self.cloud_pub.publish(bytes(out))


def main():
    rclpy.init()
    rclpy.spin(RealTfBridge())


if __name__ == '__main__':
    main()
