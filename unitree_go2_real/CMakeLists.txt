cmake_minimum_required(VERSION 3.8)
project(unitree_go2_real)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/real_tf_bridge.py scripts/record_bag.sh scripts/lowstate_joint_states.py
           scripts/odom_check.py scripts/bag_check.py scripts/odom_lidar_check.py
           scripts/bridge_selftest.py scripts/cloud_profile.py scripts/merge_sweep.py
           scripts/smear_check.py scripts/slam_track.py scripts/map_report.py
           scripts/sweep_truth.py scripts/traj_score.py scripts/odom_tums.py scripts/odom_to_tum.py
           scripts/slam_tum.py scripts/kiss_icp_bag.py scripts/lio_input.py scripts/go2_cmd_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
