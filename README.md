# Unitree Go2 ROS2

<p align="center">
  <img src="https://oss-global-cdn.unitree.com/static/c487f93e06954100a44fac4442b94d94_288x238.png" alt="Unitree Go2" />
</p>

## Overview

This package provides a complete ROS 2 Jazzy integration for the Unitree Go2 quadrupedal robot using the CHAMP controller framework. It includes custom configuration packages and robot description models specifically adapted for ROS 2, enabling simulation, control, and autonomous operation capabilities.

> **New to ROS?** Two documents, in this order:
>
> 1. [docs/STAGE_BY_STAGE.md](docs/STAGE_BY_STAGE.md) — **understand it.** The
>    robot as it came, then each layer added on top (lidar to `/scan`, SLAM,
>    AMCL, Nav2), and for every file whether it was written, copied from an
>    installed ROS package, or edited — with the exact changes and why.
> 2. [docs/NAV2_SETUP_GUIDE.md](docs/NAV2_SETUP_GUIDE.md) — **rebuild it.**
>    Every commit in order with full file contents, plus the problems we hit
>    and how they were fixed.

## Quick start: navigation demo

From a fresh container (`~/go2_docker/run.sh`), inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ws
colcon build && source install/setup.bash
ros2 launch unitree_go2_navigation demo_nav.launch.py
```

That one command starts Gazebo, AMCL on the saved map of the default
world, and Nav2. RViz is **not** started by it — see
[Known limitations](#known-limitations) — start it by hand once the stack is
up:

```bash
rviz2 -d ~/ws/install/unitree_go2_navigation/share/unitree_go2_navigation/rviz/nav2_go2.rviz \
  --ros-args -p use_sim_time:=true
```

**Wait 90-120 seconds before doing anything.** Gazebo has to spawn the
robot and the controllers have to come up; errors printed during that
window are normal. The stack is ready when `/clock` is advancing,
`/joint_states`, `/odom` and `/scan` are publishing, and
`map -> odom -> base_footprint -> base_link` all resolve.

Then send goals from a second terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, \
   orientation: {w: 1.0}}}}"
```

Goal points worth trying in the default world (obstacles sit at
`(5,0)`, `(-3,4)`, `(0,-5)`, `(3,3)` and `(-4,-3)`):

| Goal | What it shows |
|---|---|
| `(2.0, 2.0)` | Plain traverse across open floor |
| `(-2.5, -2.0)` | Longer route between two obstacles |
| `(6.5, 0.0)` | Forces a detour around the box at `(5,0)` |

A successful run: the robot walks a smooth path, the global plan is
redrawn when something blocks it, the action returns `SUCCEEDED`, and the
robot ends up near the goal. Expect it to stop **0.3-2.1 m** from the
requested point - see [Known limitations](#known-limitations).

To map a new world instead of localizing on the saved one:

```bash
ros2 launch unitree_go2_navigation demo_nav.launch.py mode:=slam
ros2 run nav2_map_server map_saver_cli -f ~/ws/my_map   # when done
```

Useful arguments: `mode:=slam|localization`, `map:=/path/to/map.yaml`,
`world:=/path/to/world.sdf`, `gui:=false`, `rviz:=true` (not recommended),
`odom_source:=ekf|lio` (only `ekf` is implemented).

For the maze world (`maze.sdf`, map `maze_world2`), including manual mapping with teleop and
paste-ready navigation commands, see [docs/COMMANDS.md](docs/COMMANDS.md).

## About Unitree Go2

The Go2 is a quadrupedal robot manufactured by Unitree Robotics, designed for both research and commercial applications. It features powerful actuators, advanced sensor integration, and a robust mechanical design capable of navigating various terrains.

## About CHAMP Controller

CHAMP (Coupled Hybrid Automata for Mobile Platforms) is an open-source development framework designed for quadrupedal robots. It provides a hierarchical control system that combines pattern modulation and impedance control techniques for efficient locomotion.

![CHAMP Controller](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/robots.gif)

## Features

- ✅ Complete ROS 2 Jazzy integration
- ✅ URDF model adapted to ROS 2 control framework
- ✅ Gazebo Harmonic simulation support
- ✅ Teleoperation using keyboard
- ✅ RVIZ visualization
- ✅ Integrated gait control and configuration
- ✅ Simulated sensors (in progress):
  - ✅ IMU
  - ✅ 2D LiDAR (Hokuyo)
  - ✅ 3D LiDAR (Velodyne)
  - ✅ 3D LiDAR (4D Lidar L1) (need some imporvments)
  - ✅ Mono Camera
  - ✅ Depth Camera (RealSense D455)
  - ✅ GPS (NavSat — simulates standard u-blox, ~0.5 m horizontal accuracy)
- ✅ Point cloud visualization in RVIZ
- ✅ Multiple sensor configurations available
- ✅ 2D SLAM with slam_toolbox (lidar flattened to `/scan`)
- ✅ Navigation 2 integration (AMCL localization or live mapping)

## System Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim Harmonic

## Installation

### 1. Install ROS 2 Dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-gazebo-ros2-control
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-velodyne
sudo apt install ros-jazzy-velodyne-description
```

### 2. Clone and Install CHAMP Controller and Go2 Simulation Packages

```bash
cd ~/ros2_ws/src
git clone https://github.com/khaledgabr77/unitree_go2_ros2
```

### 3. Install Dependencies

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage

### Gazebo Simulation

Launch the Gazebo simulation:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py
```

![Unitree Go2 Simulation](docs/unitree_go2_sim.png)

#### Joint command interface

The joints are driven through `ros2_control`. By default the effort
interface is used, matching `joint_group_effort_controller` in
`unitree_go2_sim/config/ros_control/ros_control.yaml`. If needed,
can be changed to position interface with:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py command_interface:=position
```

Remember to switch the spawned controller in `ros_control.yaml` to a
position controller as well, otherwise the controller manager will fail to
claim the joints.

#### Leaving sensors out

All sensors are mounted by default. Each one can be left out of the robot
description, which is useful when running headless or on a machine that
struggles with the point cloud sensors:

| Argument | Sensor |
|----------|--------|
| `disable_camera` | mono RGB camera |
| `disable_d455` | RealSense D455 RGBD camera |
| `disable_lidar_l1` | 4D Lidar L1 |
| `disable_velodyne_lidar` | Velodyne VLP-16 |

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py \
    disable_velodyne_lidar:=true disable_d455:=true
```

[Watch Demo on YouTube](https://youtu.be/NUu7TaZhaQM)

### RVIZ Visualization

The package now includes both `Velodyne 3D LiDAR` and `4D Lidar L1` sensors. You can visualize the point cloud data in RVIZ:

Launch Gazebo with RVIZ:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=true
```

![RVIZ Visualization](docs/unitree_go2_vis.png)

**Velodyne Lidar and 4D Lidar L1**

![Velodyne Lidar and 4D Lidar L1](docs/1.png)

**Velodyne Lidar Beams**

![Velodyne Lidar Beams](docs/2.png)

**4D Lidar L1 Beams**

![4D Lidar L1 Beams](docs/3.png)

**Velodyne Lidar and 4D Lidar L1 Beams**

![Velodyne Lidar and 4D Lidar L1 Beams](docs/4.png)

**Mono Camera**

![Mono Camera](docs/camera.png)

### Teleoperation

Control the robot using keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### GPS Simulation

The robot includes a simulated GPS (NavSat) sensor mounted on top of the trunk, publishing at 10 Hz on `/gps/fix` as `sensor_msgs/msg/NavSatFix`.

The sensor models standard u-blox-level accuracy:

| Axis | Noise (std dev) | Equivalent variance |
|------|----------------|---------------------|
| Horizontal (lat/lon) | 0.5 m | 0.25 m² |
| Vertical (altitude) | 0.8 m | 0.64 m² |

**Verify it is working:**

```bash
ros2 topic echo /gps/fix
```

**Setting the geographic origin:**

The world's GPS origin is defined in `unitree_go2_description/worlds/default.sdf` under `<spherical_coordinates>`. Edit `latitude_deg`, `longitude_deg`, and `elevation` to match your actual test site before running navigation experiments:

```xml
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>YOUR_LAT</latitude_deg>
  <longitude_deg>YOUR_LON</longitude_deg>
  <elevation>YOUR_ALT_METERS</elevation>
  <heading_deg>0</heading_deg>
</spherical_coordinates>
```

> **Note:** The `position_covariance` field in the bridged message will be all-zeros (`COVARIANCE_TYPE_UNKNOWN`) because the Gazebo→ROS bridge does not carry covariance. When integrating with `robot_localization`, set the covariance override in your EKF config or add a relay node that stamps diagonal covariance values matching the noise above.

## Mapping and navigation

The `unitree_go2_navigation` package flattens the L1 4D lidar cloud into a
2D `/scan` and feeds it to slam_toolbox or Nav2.

| Launch file | What it does |
|---|---|
| `slam.launch.py` | Sim + scan conversion + slam_toolbox. Mapping only, no Nav2 |
| `nav2.launch.py` | Adds Nav2. `mode:=slam` or `mode:=localization` |
| `demo_nav.launch.py` | `nav2.launch.py` plus a Nav2 RViz view. Use this for demos |

The lidar is mounted tilted, so the cloud is transformed into
`base_footprint` before a height band is applied; filtering in the sensor
frame would not mean "above the ground". Measured in `base_footprint` the
ground sits near `-0.05 m`, so the band is `0.25 - 1.1 m`
(`config/pointcloud_to_laserscan.yaml`).

### TF ownership

Exactly one node owns each link of the chain. Do not add a second
publisher for any of them:

| Transform | Owner |
|---|---|
| `map -> odom` | slam_toolbox (mapping) **or** AMCL (localization) |
| `odom -> base_footprint` | `footprint_to_odom_ekf` (robot_localization) |
| `base_footprint -> base_link` | `base_to_footprint_ekf` (robot_localization) |

The simulation normally publishes a static identity `map -> odom`. All of
the navigation launch files pass `disable_static_map_odom:=true` so SLAM
or AMCL can own it instead. If you launch `unitree_go2_sim` directly and
then start SLAM by hand, you will get two publishers fighting over
`map -> odom`.

### Velocity path

Nav2 must not talk to the robot directly. The chain is:

```
controller_server -> /cmd_vel_nav -> velocity_smoother
                  -> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel
```

`collision_monitor` is the only publisher on `/cmd_vel`, which is what
CHAMP's `quadruped_controller_node` subscribes to. Nav2 is brought up node
by node rather than through `nav2_bringup/navigation_launch.py`, because
that also starts `docking_server`, which would publish `/cmd_vel` as well.
Velocity and acceleration limits in `config/nav2_params.yaml` follow
`unitree_go2_sim/config/gait/gait.yaml`; raising them makes the robot tip
over.

## Known limitations

**Localization accuracy is poor in the default world.** The world contains
only five small obstacles, so AMCL has very little to match against and
the robot typically stops **0.3-1.5 m** from the requested goal, with the
larger errors (up to 2.1 m seen) when it travels through unmapped space. Nav2 reports
`SUCCEEDED` because it believes it arrived. A world with walls would fix
this; the navigation stack itself is not at fault.

**The lidar cannot see anything closer than 0.8 m.** That is the sensor's
own minimum range, and the robot is about 0.7 m long, so obstacles right
against the robot are invisible. Costmap inflation is set to 0.70 m to
compensate, but do not expect reliable avoidance of something that walks
into the robot.

**Starting RViz from the launch file breaks the stack.** DDS discovery
saturates, `/clock` stops being delivered to ROS even though Gazebo keeps
running, and Nav2 never activates. Delaying RViz by 45 s made this
intermittent rather than fixing it, so `demo_nav.launch.py` now defaults
to `rviz:=false`. Start RViz by hand once the stack is up, which was
reliable in testing. If `/clock` is already lost, closing RViz does not
recover it - restart the stack.

**MPPI is too slow here.** It could not hold its 20 Hz control rate
alongside Gazebo (measured ~5.5 Hz) and commanded only ~0.06 m/s, which
tripped Nav2's progress checker. The controller is DWB instead.

**Lidar timing.** The lidar publishes 10 frames per *simulated* second as
configured. Its wall-clock rate looks lower (~8.3-9.3 Hz) only because the
simulation runs at roughly 0.85-0.95x real time. Measured over 12 minutes
both headless and with the GUI, the gap between `/clock` and the lidar's
header stamp stays flat at **0.0-0.12 s** and does not accumulate. If you
ever do see it grow without bound, suspect two simulators running at once
rather than the sensor.

**Only one simulator at a time.** Two Gazebo instances both publish
`/clock`, time appears to jump backwards, the leg controller dies with
`Pid is called with negative dt`, and every node spams
`Detected jump back in time`. Check with `pgrep -af "gz sim"` before
relaunching.

## Tuning Gait Parameters

The gait configuration for the robot is found in `unitree_go2_sim/config/gait/gait.yaml`. You can modify the following parameters:

| Parameter | Description |
|-----------|-------------|
| Knee Orientation | How the knees should be bent (.>> .>< .<< .<>) |
| Max Linear Velocity X | Maximum forward/reverse speed (m/s) |
| Max Linear Velocity Y | Maximum sideways speed (m/s) |
| Max Angular Velocity Z | Maximum rotational speed (rad/s) |
| Stance Duration | How long each leg spends on the ground while walking |
| Leg Swing Height | Trajectory height during swing phase (m) |
| Leg Stance Height | Trajectory depth during stance phase (m) |
| Robot Walking Height | Distance from hip to ground while walking (m) |
| CoM X Translation | Offset to compensate for weight distribution |
| Odometry Scaler | Multiplier to calculated velocities for dead reckoning |

![Velodyne Lidar and 4D Lidar L1](docs/image.png)

## Project Structure

- `champ/`: Core controllers and state estimation for CHAMP
- `unitree_go2_description/`: URDF models, meshes, and world files
- `unitree_go2_sim/`: Simulation launch files and configuration
- `unitree_go2_navigation/`: Scan conversion, slam_toolbox, Nav2 and the demo launch

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'feat: Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## Acknowledgements

This project builds upon and incorporates work from the following projects:

* [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros) - For the Go2 robot description (URDF model)
* [CHAMP](https://github.com/chvmp/champ) - For the quadruped controller framework
* [CHAMP Robots](https://github.com/chvmp/robots) - For robot configurations and setup examples
* [unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2) - ROS 2 package with gazebo classic

## License
