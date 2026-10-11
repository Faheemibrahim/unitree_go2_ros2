# Go2 maze: mapping and navigation commands

Paste-ready commands. Everything runs inside the `go2_jazzy` container unless marked
**host**. The workspace is `~/go2_jazzy_ws` on the host and `~/ws` in the container.

There are three launch files, each started in its own terminal:

| What | Command | Opens |
|---|---|---|
| Simulator | `ros2 launch unitree_go2_sim unitree_go2_launch.py` | Gazebo only (maze world) |
| Mapping | `ros2 launch unitree_go2_navigation slam.launch.py` | slam_toolbox + RViz mapping view |
| Navigation | `ros2 launch unitree_go2_navigation nav2.launch.py` | AMCL + Nav2 + RViz Nav2 view |

Always start the simulator first and wait for its `[sim] Robot is up` line (about 15 s). Run **either** SLAM **or**
Nav2, never both: each one owns the `map -> odom` transform.

- Part 1 builds a map by driving the robot yourself.
- Part 2 navigates on a saved map (`maze_world2`).

Start both parts from a **fresh** simulator if you want the map frame to line up with
Gazebo's world: SLAM's map origin is wherever the robot's odometry is when it starts.

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

### Terminal A: the simulator

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py
```

Defaults: the maze world, Gazebo window on, no RViz, and no static `map -> odom` (slam_toolbox
or AMCL owns it). The controllers are spawned one after another as each step finishes; wait
for the line `[sim] Robot is up: leg controllers active` (about 15 s), then a few seconds for the
robot to stand. Other worlds: `world:=/path/to/world.sdf`. No window: `gui:=false`.

### Terminal B: SLAM (after the simulator has settled)

```bash
ros2 launch unitree_go2_navigation slam.launch.py
```

This also opens RViz with the mapping view, as soon as slam_toolbox is active. Add `rviz:=false` to leave it out. Restart only this
terminal after changing `slam_toolbox.yaml`.

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

### Terminal A: the simulator

Same as Part 1, Terminal A:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py
```

Wait for `[sim] Robot is up` (about 15 s), then a few seconds for the robot to stand.

### Terminal B: Nav2 + AMCL on maze_world2, with RViz

```bash
ros2 launch unitree_go2_navigation nav2.launch.py
```

The stages start one after another, each when the previous one reports its nodes active:
map_server + AMCL (3 s after the launch), then Nav2, then RViz with the Nav2 view. Everything is
up in about 10 s (`rviz:=false` to leave RViz out). If RViz does not open, a stage stalled. The map defaults to `maze_world2.yaml`; use `map:=/path/to/other.yaml` for another map of the
same world. Check it is ready:

```bash
ros2 topic hz /scan                      # about 8-10 Hz
ros2 lifecycle get /bt_navigator         # should be: active
```

If the nodes stay `unconfigured`, Nav2 bringup stalled (a lifecycle reply was dropped under
CPU load). RViz will not open in that case. Ctrl+C and relaunch.

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
detect them. `unmapped_obstacle_marker.py` (started by `nav2.launch.py`)
publishes a halo around scan hits that are far from every mapped wall to `/unmapped_obstacles`,
which the global costmap marks, so the planner treats a too-narrow gap beside a box as closed.
Check it with `ros2 topic echo /unmapped_obstacles --once`: it should be empty with only walls
in view. This detour behaviour was checked in isolation but a full recorded drive past a box
has not been completed yet.

### Where the map and world are set

There is one maps directory, `unitree_go2_navigation/maps/`. It holds `maze_world2.{pgm,yaml}`.
The map is the `map` argument of `nav2.launch.py`, and the world is the `world` argument of the simulator:

- **Per run:** pass `map:=/path/to/file.yaml` to Nav2 and/or `world:=/path/to/world.sdf` to the simulator.
- **Map default:** `default_map` near the top of
  [nav2.launch.py](../unitree_go2_navigation/launch/nav2.launch.py).
- **World default:** `default_world_path` near the top of
  [unitree_go2_launch.py](../unitree_go2_sim/launch/unitree_go2_launch.py), or `world:=` on the
  simulator command.
- **Keep the map and world as a pair.** A map made in one world will not localize in another.

After editing a launch file, or adding a map, rebuild `unitree_go2_navigation` so it is installed.

To map something new, save into the same directory with a new name, for example
`maze_world3`, then pass `map:=` (see Part 1, Terminal D).

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
- Map not found at launch: rebuild `unitree_go2_navigation` so the map is installed, and check the
  file is in `unitree_go2_navigation/maps/` (there is only that one maps directory).
- Robot localizes badly or goals land off target: check `odom_scaler` in
  `unitree_go2_sim/config/gait/gait.yaml` first. Leg odometry that under-reports distance made
  AMCL fall behind, which looked like a planning problem. AMCL's laser range is already set to
  the L1's 30 m; the lidar cannot see closer than 0.8 m.
- Robot hugs or clips wall ends: the maze has 0.85 m gaps at wall ends and the body clears them
  by only a few cm. Slow tight curves are intended (`regulated_linear_scaling_min_radius`).
