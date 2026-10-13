cmake_minimum_required(VERSION 3.8)
project(unitree_go2_real)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/real_tf_bridge.py scripts/record_bag.sh scripts/lowstate_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
