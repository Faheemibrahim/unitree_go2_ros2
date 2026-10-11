"""Lidar cloud to 2D /scan: crop the robot's own legs out of the L1 cloud, then flatten
a height band around the lidar into a LaserScan.

  /unitree_lidar/points -> body_crop -> /unitree_lidar/points_cropped
                        -> pointcloud_to_laserscan -> /scan

nav2.launch.py and slam.launch.py include this. On its own it shows what the 2D scan
looks like (open unitree_go2_sim/rviz/lidar_check.rviz):
  ros2 launch unitree_go2_navigation scan.launch.py

Both nodes are stock C++ packages (pcl_ros, pointcloud_to_laserscan), so the same
pipeline can run on the robot's Jetson. Values and their reasons are in
config/pointcloud_to_laserscan.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("unitree_go2_navigation"),
        "config",
        "pointcloud_to_laserscan.yaml",
    )

    body_crop_node = Node(
        package="pcl_ros",
        executable="filter_crop_box_node",
        name="body_crop",
        output="screen",
        parameters=[config],
        remappings=[
            ("input", "/unitree_lidar/points"),
            ("output", "/unitree_lidar/points_cropped"),
        ],
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[config],
        remappings=[
            ("cloud_in", "/unitree_lidar/points_cropped"),
            ("scan", "/scan"),
        ],
    )

    return LaunchDescription([body_crop_node, pointcloud_to_laserscan_node])
