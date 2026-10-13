#!/usr/bin/env bash
# Records the real Go2's own topics into ~/ws/bags/<name> (host: ~/go2_jazzy_ws/bags/<name>).
# Listen-only: it sends nothing to the robot. Stop with Ctrl+C.
#
#   ros2 run unitree_go2_real record_bag.sh [name]      (inside `./run.sh real`)
#
# The robot's topics are stored as they arrive, robot-clock stamps included; the bridge
# (real_tf_bridge.py) is re-run on replay, so TF and PC-time copies are not stored twice.
# ~9 MB/s, mostly the three clouds: ~1.3 GB for 2.5 minutes.
set -e

if [ "${ROS_DOMAIN_ID:-0}" != "0" ] || [ "$RMW_IMPLEMENTATION" != "rmw_cyclonedds_cpp" ]; then
  echo "Not in real-robot mode (ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, RMW=${RMW_IMPLEMENTATION})." >&2
  echo "Start the container with: ~/go2_docker/run.sh real" >&2
  exit 1
fi

NAME="${1:-go2_$(date +%Y%m%d_%H%M%S)}"
OUT="$HOME/ws/bags/$NAME"
mkdir -p "$(dirname "$OUT")"

TOPICS=(
  # Mapping input and odometry
  /utlidar/cloud_deskewed   # L1 cloud in odom, 15 Hz: the mapping input
  /utlidar/cloud            # raw L1 cloud in utlidar_lidar, 15 Hz
  /utlidar/cloud_base       # L1 cloud in base_link, 15 Hz
  /utlidar/robot_odom       # odometry odom -> base_link, 150 Hz
  /utlidar/robot_pose       # same pose, 19 Hz
  /utlidar/imu              # lidar IMU, 250 Hz
  /utlidar/lidar_state      # lidar status, 5 Hz
  # Body state
  /sportmodestate           # mode, gait, body velocity, foot positions, 300 Hz
  /lf/sportmodestate        # same at 20 Hz
  /lowstate                 # joints, IMU, foot force, battery, 500 Hz
  # What drove the robot, and what the onboard services were doing
  /wirelesscontroller       # remote sticks/buttons
  /wirelesscontroller_unprocessed
  /api/sport/request        # sport commands (none expected when driving by remote)
  /api/sport/response
  /api/obstacles_avoid/request   # onboard obstacle avoidance, 13 Hz, tiny
  /api/obstacles_avoid/response
  /api/motion_switcher/request
  /api/motion_switcher/response
  /multiplestate
  /servicestate
  # Camera (0.2 MB/s), to see the lab next to the map
  /frontvideostream
  # Onboard maps / SLAM: idle on this robot so far; recorded in case they wake up
  /utlidar/grid_map
  /utlidar/height_map
  /utlidar/height_map_array
  /utlidar/range_map
  /utlidar/range_info
  /utlidar/voxel_map
  /uslam/cloud_map
  /uslam/localization/odom
  /uslam/frontend/odom
  /lio_sam_ros2/mapping/odometry
)

echo "Recording ${#TOPICS[@]} topics to $OUT (Ctrl+C to stop)"
exec ros2 bag record -s mcap -o "$OUT" "${TOPICS[@]}"
