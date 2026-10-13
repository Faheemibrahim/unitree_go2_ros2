"""Watch a recorded Go2 bag in RViz: robot model, TF, clouds, odometry trail, 2D scan. No SLAM.

  ros2 launch unitree_go2_real real_playback.launch.py
  ros2 bag play ~/ws/bags/<bag>        (second terminal, same domain; play at 1x)

Bag playback needs no robot and no robot network: in the shell, `unset CYCLONEDDS_URI`
and use another ROS_DOMAIN_ID (e.g. 50) so replayed topics do not mix with a live robot.

  bag topics -> real_tf_bridge (TF odom -> base_footprint -> base_link, PC-time copies
                /go2/odom, /go2/cloud_deskewed, plus _raw copies; yaw_scale in config/real_bridge.yaml)
  /lowstate -> lowstate_joint_states -> /joint_states -> robot_state_publisher
  /go2/cloud_deskewed -> body_crop -> cloud_accumulator -> pointcloud_to_laserscan -> /scan
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    real = get_package_share_directory("unitree_go2_real")
    scan_config = os.path.join(real, "config", "pointcloud_to_laserscan.yaml")
    bridge_config = os.path.join(real, "config", "real_bridge.yaml")
    rviz_config = os.path.join(real, "rviz", "playback.rviz")

    # The official model's root link is `base`; the robot's odometry uses `base_link`.
    urdf_file = os.path.join(
        get_package_share_directory("go2_description"), "urdf", "go2_description.urdf"
    )
    with open(urdf_file) as f:
        urdf = f.read().replace(
            "</robot>",
            '<link name="base_link"/><joint name="base_link_to_base" type="fixed">'
            '<parent link="base_link"/><child link="base"/></joint></robot>',
        )

    no_sim_time = {"use_sim_time": False}

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz"),
        Node(package="unitree_go2_real", executable="real_tf_bridge.py",
             name="real_tf_bridge", output="screen", parameters=[bridge_config]),
        Node(package="unitree_go2_real", executable="lowstate_joint_states.py",
             name="lowstate_joint_states", output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf}, no_sim_time]),
        Node(package="pcl_ros", executable="filter_crop_box_node", name="body_crop",
             output="screen", parameters=[scan_config],
             remappings=[("input", "/go2/cloud_deskewed"), ("output", "/l1_cloud_cropped")]),
        Node(package="pcl_ros", executable="filter_passthrough_node", name="band",
             output="screen", parameters=[scan_config],
             remappings=[("input", "/l1_cloud_cropped"), ("output", "/l1_cloud_band")]),
        Node(package="unitree_go2_navigation", executable="cloud_accumulator.py",
             name="cloud_accumulator", output="screen", parameters=[scan_config],
             remappings=[("input", "/l1_cloud_band"), ("output", "/l1_cloud_merged")]),
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
             name="pointcloud_to_laserscan", output="screen", parameters=[scan_config],
             remappings=[("cloud_in", "/l1_cloud_merged"), ("scan", "/scan")]),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", rviz_config], parameters=[no_sim_time],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
