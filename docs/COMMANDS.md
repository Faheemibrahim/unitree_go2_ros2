# Go2 maze: mapping and navigation commands

Paste-ready commands. Everything runs inside the `go2_jazzy` container unless marked
**host**. The workspace is `~/go2_jazzy_ws` on the host and `~/ws` in the container.

- Part 1 builds a map by driving the robot yourself.
- Part 2 navigates on a saved map (maze_world2) with Nav2.

Do not run Part 1 and Part 2 at the same time. Part 2 starts its own sim.

## 0. Setup

### Clear stale shared-memory files (host, only when no sim is running)

The container uses `--ipc=host`, so leftovers from old sessions pile up and can leave RViz
seeing no topics.

```bash
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

### Start the container (host)

```bash
cd ~/go2_docker && ./run.sh
```

Open each extra terminal with `cd ~/go2_docker && ./run.sh shell`. In **every** container
terminal, source ROS first:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ws/install/setup.bash
```

### Build (after editing any file, or after adding a map or world)

```bash
cd ~/ws && colcon build --packages-select unitree_go2_description unitree_go2_sim unitree_go2_navigation
source install/setup.bash
```

Maps and worlds are copied into `install/` at build time, so a new map such as
`maze_world2` is not found until you rebuild.

---

## Part 1: Mapping (manual drive)

### Terminal A: sim + RViz with the maze world

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py \
  world:=$(ros2 pkg prefix unitree_go2_description)/share/unitree_go2_description/worlds/maze.sdf \
  disable_static_map_odom:=true
```

Wait 90-120 s, until the log shows `joint_group_effort_controller` configured and activated.
`disable_static_map_odom:=true` is needed because slam_toolbox publishes `map -> odom`.

### Terminal B: SLAM (after the sim has settled)

```bash
ros2 launch unitree_go2_navigation slam_only.launch.py
```

Restart only this terminal after changing `slam_toolbox.yaml`.

### Terminal C: drive

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Tap keys, do not hold them, and drive slowly. Abrupt velocity changes make the robot fall,
and a fallen robot corrupts the scans.

### Terminal D: save the map

```bash
cd ~/ws/src/unitree_go2_ros2/unitree_go2_navigation/maps
ros2 run nav2_map_server map_saver_cli -f maze_world2
```

This writes `maze_world2.pgm` and `maze_world2.yaml`. Rebuild afterwards (see Setup) so the
map is installed.

Stop Part 1 with Ctrl+C in every terminal before starting Part 2.

---

## Part 2: Navigation on a saved map

### Terminal A: sim + Nav2 + AMCL on maze_world2

```bash
ros2 launch unitree_go2_navigation demo_nav.launch.py \
  mode:=localization \
  world:=$(ros2 pkg prefix unitree_go2_description)/share/unitree_go2_description/worlds/maze.sdf \
  map:=$(ros2 pkg prefix unitree_go2_navigation)/share/unitree_go2_navigation/maps/maze_world2.yaml
```

Wait 90-120 s (Nav2 itself starts at 40 s). Then check it is ready:

```bash
ros2 topic hz /scan                      # about 8-10 Hz
ros2 lifecycle get /bt_navigator         # should be: active
```

If the nodes stay `unconfigured`, Nav2 bringup stalled (known DDS issue). Ctrl+C and relaunch.

### Terminal B: Nav2 RViz (start it by hand once Terminal A is up)

```bash
ros2 run rviz2 rviz2 \
  -d $(ros2 pkg prefix unitree_go2_navigation)/share/unitree_go2_navigation/rviz/nav2_go2.rviz \
  --ros-args -p use_sim_time:=true
```

RViz is not started by the launch file on purpose: starting it from there saturated DDS
discovery on this machine.

### Move the robot

In RViz:

1. Check the robot sits on the map where it is in Gazebo. The AMCL start pose is fixed at
   (0, 0), which matches the spawn point. If it is wrong, click **2D Pose Estimate** and drag
   on the map.
2. Click **2D Goal Pose** and drag on a free cell to send a goal.

Or send a goal from a terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Do not run teleop during navigation. `/cmd_vel` must have exactly one publisher
(`collision_monitor`).

A goal that has been driven successfully in simulation, from the spawn (map frame, about 84 m,
about 11 minutes): `x: -10.49, y: 9.37`. Start with something short before trying it.

### Obstacles that are not in the map

The four 0.5 m boxes in `maze.sdf` are deliberately not in `maze_world2`. The robot has to
detect them. `unmapped_obstacle_marker.py` (started by `nav2.launch.py` in localization mode)
publishes a halo around scan hits that are far from every mapped wall to `/unmapped_obstacles`,
which the global costmap marks, so the planner treats a too-narrow gap beside a box as closed.
Check it with `ros2 topic echo /unmapped_obstacles --once`: it should be empty with only walls
in view. This detour behaviour was checked in isolation but a full recorded drive past a box
has not been completed yet.

### Where to change the map path

The map is the `map` launch argument. Two ways to set it:

- **Per run (used above):** pass `map:=/path/to/file.yaml` on the command line.
- **Permanently:** change the default in the launch file.
  - [nav2.launch.py](../unitree_go2_navigation/launch/nav2.launch.py), line 37:
    `default_map = os.path.join(unitree_go2_navigation, "maps", "default_world.yaml")`
  - [demo_nav.launch.py](../unitree_go2_navigation/launch/demo_nav.launch.py), line 25: same line.
  - Change `default_world.yaml` to `maze_world2.yaml` in **both**. `demo_nav` passes its own
    default into `nav2.launch.py`, so changing only one has no effect when you launch `demo_nav`.

The world works the same way: `default_world` on line 38 of `nav2.launch.py` and line 26 of
`demo_nav.launch.py` (change `default.sdf` to `maze.sdf`). Rebuild `unitree_go2_navigation`
after any edit.

---

## Quick checks if something looks wrong

```bash
ros2 topic hz /scan                                         # about 8-10 Hz
ros2 lifecycle get /slam_toolbox                            # Part 1: active
ros2 lifecycle get /bt_navigator                            # Part 2: active
ros2 topic echo /foot_contacts --once                       # some entries true while walking
ros2 topic echo /odom --once --field pose.pose.position     # changes as the robot walks
```

- `/foot_contacts` all `false`, or `/odom` stuck at 0: the world is missing
  `gz-sim-contact-system`. `maze.sdf` includes it.
- RViz sees no topics: clear `/dev/shm` (Setup), then relaunch.
- Robot missing in the mapping RViz before SLAM starts: Fixed Frame must be `odom`
  (`unitree_go2_sim/rviz/rviz.rviz`).
- Map not found at launch: rebuild `unitree_go2_navigation` so the map is installed.
- Robot localizes badly or goals land off target: check `odom_scaler` in
  `unitree_go2_sim/config/gait/gait.yaml` first. Leg odometry that under-reports distance made
  AMCL fall behind, which looked like a planning problem. AMCL's laser range is already set to
  the L1's 30 m; the lidar cannot see closer than 0.8 m.
- Robot hugs or clips wall ends: the maze has 0.85 m gaps at wall ends and the body clears them
  by only a few cm. Slow tight curves are intended (`regulated_linear_scaling_min_radius`).
