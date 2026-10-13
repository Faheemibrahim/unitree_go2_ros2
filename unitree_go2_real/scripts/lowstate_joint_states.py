#!/usr/bin/env python3
"""/lowstate (500 Hz) -> /joint_states, so robot_state_publisher can pose the official
Go2 model's legs in RViz. Listen-only.

Motor order is Unitree's (unitree_ros2 example/src/include/common/motor_crc.h):
0-2 FR, 3-5 FL, 6-8 RR, 9-11 RL, each hip, thigh, calf. Joint names are the official
URDF's (go2_description). Published at rate_hz with the PC clock, because LowState has
no header and RViz only needs the latest leg pose.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState

JOINTS = [f'{leg}_{part}_joint' for leg in ('FR', 'FL', 'RR', 'RL')
          for part in ('hip', 'thigh', 'calf')]


class LowStateJointStates(Node):

    def __init__(self):
        super().__init__('lowstate_joint_states')
        self.latest = None
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(LowState, '/lowstate', self.on_lowstate, qos_profile_sensor_data)
        self.create_timer(1.0 / self.declare_parameter('rate_hz', 50.0).value, self.publish)

    def on_lowstate(self, msg):
        self.latest = msg.motor_state[:12]

    def publish(self):
        if self.latest is None:
            return
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = JOINTS
        out.position = [float(m.q) for m in self.latest]
        out.velocity = [float(m.dq) for m in self.latest]
        out.effort = [float(m.tau_est) for m in self.latest]
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(LowStateJointStates())


if __name__ == '__main__':
    main()
