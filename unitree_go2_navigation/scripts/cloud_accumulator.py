#!/usr/bin/env python3
"""Merges the lidar clouds of the last window_s seconds into one.

One L1 message is sparse: about 1,400 points on 12 mirror sweeps, with 4-22 deg gaps
in azimuth between them. Successive messages sweep in between, so ~0.3 s together
cover the scene (docs/L1_LIDAR_REALISM.md section 4.3; Go2 users buffer ~5 messages,
section 7). The input must be in a fixed frame (odom), so points from earlier messages
stay where they were measured while the robot moves; on the Go2 that is
/utlidar/cloud_deskewed. The merged cloud carries the newest stamp.

Runs on the robot as well, so it stays cheap: clouds with the same layout are joined
byte for byte, no per-point work.
"""

import array
import collections

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2


class CloudAccumulator(Node):

    def __init__(self):
        super().__init__('cloud_accumulator')
        self.window = self.declare_parameter('window_s', 0.3).value
        self.clouds = collections.deque()
        self.create_subscription(PointCloud2, 'input', self.on_cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, 'output', qos_profile_sensor_data)

    def on_cloud(self, msg):
        if msg.height != 1 and msg.width:
            msg.width, msg.height = msg.width * msg.height, 1
        if self.clouds and (msg.header.frame_id != self.clouds[-1].header.frame_id
                            or msg.point_step != self.clouds[-1].point_step):
            self.get_logger().warn('cloud frame or layout changed; starting over')
            self.clouds.clear()
        newest = Time.from_msg(msg.header.stamp)
        if self.clouds and newest < Time.from_msg(self.clouds[-1].header.stamp):
            self.clouds.clear()             # time went backwards (sim reset)
        self.clouds.append(msg)
        while (newest - Time.from_msg(self.clouds[0].header.stamp)).nanoseconds * 1e-9 > self.window:
            self.clouds.popleft()

        out = PointCloud2()
        out.header = msg.header
        out.height, out.width = 1, sum(c.width for c in self.clouds)
        out.fields, out.is_bigendian, out.point_step = msg.fields, msg.is_bigendian, msg.point_step
        out.row_step = out.point_step * out.width
        out.is_dense = all(c.is_dense for c in self.clouds)
        # An array.array is taken as is; bytes would be checked element by element in
        # Python (rosidl's setter), which cost ~50% of a core at 15 Hz.
        out.data = array.array('B', b''.join(c.data.tobytes() for c in self.clouds))
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(CloudAccumulator())


if __name__ == '__main__':
    main()
