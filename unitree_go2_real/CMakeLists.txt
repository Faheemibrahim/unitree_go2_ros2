cmake_minimum_required(VERSION 3.8)
project(unitree_go2_real)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/real_tf_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
