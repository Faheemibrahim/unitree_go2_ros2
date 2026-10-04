# Nav2 Setup Guide — rebuilding this from scratch

This guide rebuilds the navigation work in this repository from nothing, in the
order it was actually built. It is written for someone who has **never used
ROS**. Every term is defined in the [Glossary](#7-glossary), every command can
be copied and pasted, and every command says what you should see.

Everything here was taken from the real files and commits in this repository.
Where something was **not** verified, it says so plainly.

---

## Table of contents

1. [What this is, in plain language](#1-what-this-is-in-plain-language)
2. [What you need before starting](#2-what-you-need-before-starting)
3. [Build-order walkthrough](#3-build-order-walkthrough)
4. [Problems we hit, and the fixes](#4-problems-we-hit-and-the-fixes)
5. [Running the demo from a fresh container](#5-running-the-demo-from-a-fresh-container)
6. [What is verified, and what is not](#6-what-is-verified-and-what-is-not)
7. [Glossary](#7-glossary)
8. [File tree of everything added](#8-file-tree-of-everything-added)

---

## 1. What this is, in plain language

We are working with a **simulated** robot dog (a Unitree Go2) inside a physics
simulator called Gazebo. Nothing here touches a real robot.

The robot carries a **lidar** — a spinning laser sensor that measures the
distance to everything around it, many thousands of times per second. The goal
is to let you click a point and have the robot walk there by itself, avoiding
obstacles.

Three separate jobs have to happen for that to work:

**Mapping** — the robot drives around and builds a floor plan of the world from
what the lidar sees. The software that does this is called `slam_toolbox`. The
output is a **map**: a picture where each cell is "free", "blocked", or
"unknown". You save that map to a file and reuse it.

**Localisation** — given a map you already have, work out *where on the map the
robot currently is*. The software is called `AMCL`. It compares what the lidar
sees right now against the saved map and corrects the robot's estimated
position as it walks.

**Navigation** — given a map and a current position, work out a route to the
goal and send walking commands to follow it. The software is **Nav2**.

You use mapping **or** localisation, never both at once, because they both
produce the same answer (where the robot is) and would fight each other.

### How the pieces connect

```
  Gazebo (simulated world, robot body, lidar)
      |
      |  /unitree_lidar/points       3D cloud of points, 10 per second
      v
  pointcloud_to_laserscan            squashes 3D down to a flat 2D slice
      |
      |  /scan                       a flat ring of distances
      v
  slam_toolbox  (mapping mode)  OR  AMCL  (localisation mode)
      |                                |
      |  builds /map                   |  reads saved /map
      +--------------+-----------------+
                     |
                     v
                   Nav2                plans a route, follows it
                     |
                     |  /cmd_vel_nav
                     v
              velocity_smoother        stops sudden speed changes
                     |
                     |  /cmd_vel_smoothed
                     v
             collision_monitor         last safety gate
                     |
                     |  /cmd_vel       <-- ONLY this one talks to the robot
                     v
        CHAMP quadruped_controller_node
                     |
                     v
          leg joint commands -> Gazebo
```

Everything with a `/` in front of it is a **topic** — a named channel that one
program writes to and others read from.

### The TF frames

**TF** is the system that keeps track of where things are relative to each
other. Each named position is a **frame**, and they form a chain:

```
  map                 the world's fixed origin; where the saved map lives
   |
   |  owned by slam_toolbox (mapping) or AMCL (localisation)
   v
  odom                drift-free short-term reference
   |
   |  owned by footprint_to_odom_ekf
   v
  base_footprint      the robot's position flattened onto the floor
   |
   |  owned by base_to_footprint_ekf
   v
  base_link           the robot's body (about 0.22 m above the floor)
   |
   |  fixed, from the robot's 3D model
   v
  lidar_l1_link       the lidar itself
```

**The single most important rule in this whole guide:** exactly one program may
own each link in that chain. If two programs publish the same link, the robot's
position jumps around and nothing works. Section 4 shows what that looks like
when it goes wrong.

Measured values from the running simulation, which the configuration depends on:

- `base_footprint` → `base_link` is about **0.219 m**
- `base_footprint` → `lidar_l1_link` is **(0.250, −0.038, 0.189)** metres, and
  the lidar is mounted tilted at roll ≈ 165°, yaw ≈ 90°
- the floor shows up at about **−0.05 m** in `base_footprint`

---

## 2. What you need before starting

### Hardware and software

| Thing | What we used | Why it matters |
|---|---|---|
| Computer | Linux with 32 CPU cores | Gazebo plus Nav2 is heavy; fewer cores will be slower |
| GPU | NVIDIA RTX 4060 | The simulated lidar is rendered on the GPU |
| Docker | Already installed | Everything runs inside a container, so your computer stays clean |
| Display | A normal Linux desktop | Needed for the Gazebo and RViz windows |

A **container** is a sealed box holding an operating system and all the
software needed, so you do not have to install ROS on your own machine.

### Starting the container

The container is started by a script already in the home directory. Open a
terminal and run:

```bash
~/go2_docker/run.sh
```

This script (read it with `cat ~/go2_docker/run.sh`) does the following:

- `xhost +local:docker` — lets programs inside the container open windows on
  your screen.
- `--gpus all` — gives the container access to the NVIDIA graphics card.
- `--net=host --ipc=host` — lets programs inside and outside the container talk
  to each other.
- `-v "$WS":/home/dev/ws` — this is the important one. It makes the folder
  `~/go2_jazzy_ws` on your computer appear as `/home/dev/ws` inside the
  container. **They are the same files.** Edit them outside, build them inside.
- `--rm` — when the container stops, it is deleted. Anything you installed by
  hand inside it is lost. Files under `~/go2_jazzy_ws` survive because they live
  on your computer.

**What success looks like:** the terminal prompt changes to something like
`dev@your-machine:~/ws$`. You are now "inside" the container.

### Opening more terminals inside the container

You will need two or three terminals. In a **new** terminal on your computer:

```bash
~/go2_docker/run.sh shell
```

That is the same as `docker exec -it go2_jazzy bash`. It attaches another
terminal to the already-running container.

**What success looks like:** another `dev@...:~/ws$` prompt.

### Making ROS available in a terminal

Every new terminal inside the container needs this before any `ros2` command:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash
```

`source` loads settings into the terminal. The first line makes ROS itself
available; the second makes *this project's* packages available. The second
line fails harmlessly until you have built the project at least once.

**What success looks like:** no output at all. Silence means success.

### Checking the packages you need are present

```bash
for p in slam_toolbox pointcloud_to_laserscan nav2_bringup nav2_map_server \
         realsense2_description robot_localization; do
  [ -d "/opt/ros/jazzy/share/$p" ] && echo "PRESENT $p" || echo "MISSING $p"
done
```

**What success looks like:** six lines, all starting `PRESENT`.

If any say `MISSING`, the container image needs rebuilding — see
[Problem 7](#problem-7-missing-packages-after-a-container-restart).

---

## 3. Build-order walkthrough

Each numbered step below is one real commit, in the order it was made. You can
see them yourself with:

```bash
cd ~/ws/src/unitree_go2_ros2
git log --reverse --oneline 55ff151..HEAD
```

> **A note on two large files.** `config/nav2_params.yaml` (428 lines) and
> `rviz/nav2_go2.rviz` (603 lines) were **copied from the installed
> `nav2_bringup` package** and then edited. Reproducing all 1,031 lines here
> would not help you. For those two, this guide gives you the copy command and
> then the exact edits. Every other file is shown in full.

---

### Step 1 — Let the simulator run without opening a window

**Commit:** `bc92a67` — *fix(launch): wire the gui argument to Gazebo headless mode*

**What it is for:** The simulator already had an on/off switch called `gui`, but
it was never actually wired to anything, so Gazebo always opened a window. For
automated testing we need it to run invisibly ("headless").

**File changed:** `unitree_go2_sim/launch/unitree_go2_launch.py`

```diff
-from launch.substitutions import Command, LaunchConfiguration
+from launch.substitutions import Command, LaunchConfiguration, PythonExpression

@@
         launch_arguments={
-            'gz_args': [LaunchConfiguration('world'), ' -r']  # Add -r flag to start unpaused
+            'gz_args': [
+                LaunchConfiguration('world'), ' -r',  # Add -r flag to start unpaused
+                PythonExpression([
+                    "'' if '", LaunchConfiguration('gui'),
+                    "' == 'true' else ' -s --headless-rendering'"
+                ]),
+            ]
         }.items(),
```

**What each part does:**

- `PythonExpression` lets the launch file make a decision while starting up.
- The expression reads: if `gui` is `true`, add nothing; otherwise add
  `-s --headless-rendering`.
- `-s` means "server only" — run the physics, draw no window.
- `--headless-rendering` tells Gazebo to still render the lidar on the GPU even
  with no window. Without it the lidar would produce nothing.

**Try it:**

```bash
cd ~/ws && colcon build --packages-select unitree_go2_sim
source install/setup.bash
ros2 launch unitree_go2_sim unitree_go2_launch.py gui:=false rviz:=false
```

**What you should see:** a wall of start-up text, no Gazebo window. In another
terminal, `pgrep -af "gz sim"` should show the command includes
`-r -s --headless-rendering`.

**If it does not work:** if a Gazebo window still appears, the package was not
rebuilt. `colcon build` copies files into `install/`, so source edits do nothing
until you rebuild.

---

### Step 2 — Let SLAM take over the `map` → `odom` link

**Commit:** `dc66f50` — *feat(launch): add disable_static_map_odom argument*

**What it is for:** The simulator publishes a permanent, fake `map` → `odom`
link that says "the robot never drifts". That is fine on its own, but
`slam_toolbox` and `AMCL` both need to own that link. Two owners means a broken
robot. This adds a switch to turn the fake one off.

**File changed:** `unitree_go2_sim/launch/unitree_go2_launch.py`

```diff
+    declare_disable_static_map_odom = DeclareLaunchArgument(
+        "disable_static_map_odom",
+        default_value="false",
+        description="Leave out the static map->odom transform, for when SLAM publishes it",
+    )

@@  (on the static transform publisher node)
             '--frame-id', 'map', '--child-frame-id', 'odom'
         ],
+        condition=UnlessCondition(LaunchConfiguration("disable_static_map_odom")),

@@  (in the list of launch arguments)
             declare_disable_velodyne_lidar,
+            declare_disable_static_map_odom,
```

**What each part does:**

- `DeclareLaunchArgument` creates an option you can type on the command line.
- `default_value="false"` means **nothing changes** for anyone already using
  this simulator. The fake link stays on unless you ask for it to go.
- `UnlessCondition(...)` means "run this node *unless* the switch is true".

**Try it:**

```bash
cd ~/ws && colcon build --packages-select unitree_go2_sim && source install/setup.bash
ros2 launch unitree_go2_sim unitree_go2_launch.py disable_static_map_odom:=true gui:=false rviz:=false
```

Wait **60–90 seconds**, then in another terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ws/install/setup.bash
ros2 node list | grep map_to_odom
```

**What you should see:** no output. The node is gone, which is the point.
Without the argument it would print `/map_to_odom_tf_node`.

**If it does not work:** if the node is still there, check for a *leftover* one
from an earlier run — see [Problem 4](#problem-4-two-simulators-at-once).

---

### Step 3 — Build the map (the navigation package is created)

**Commit:** `d8902d2` — *feat(navigation): add 2D mapping with pointcloud_to_laserscan and slam_toolbox*

**What it is for:** This creates the whole new package. It flattens the lidar's
3D cloud into a flat 2D `/scan`, and feeds that to `slam_toolbox` to draw a map.

**Files created:**

- `unitree_go2_navigation/package.xml`
- `unitree_go2_navigation/CMakeLists.txt`
- `unitree_go2_navigation/config/pointcloud_to_laserscan.yaml`
- `unitree_go2_navigation/config/slam_toolbox.yaml`
- `unitree_go2_navigation/launch/slam.launch.py`

First create the folders:

```bash
cd ~/ws/src/unitree_go2_ros2
mkdir -p unitree_go2_navigation/config unitree_go2_navigation/launch
```

#### `unitree_go2_navigation/package.xml`

Every ROS package needs this. It is the package's ID card and its shopping list.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>unitree_go2_navigation</name>
  <version>0.0.0</version>
  <description>Navigation and mapping for the unitree go2 robot</description>
  <maintainer email="khaledgabr77@gmail.com">khaled</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>launch_ros</build_depend>

  <exec_depend>launch_ros</exec_depend>
  <exec_depend>unitree_go2_sim</exec_depend>
  <exec_depend>pointcloud_to_laserscan</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- `<name>` must match the folder name exactly.
- `<exec_depend>` lists other software this package needs **to run**. This is
  what lets `rosdep` install the missing pieces later.

#### `unitree_go2_navigation/CMakeLists.txt`

This says how to build the package. This package has no code to compile — it is
only configuration and launch files — so all it does is copy folders into place.

```cmake
cmake_minimum_required(VERSION 3.8)
project(unitree_go2_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

> At this step the file has only `launch` and `config`. Steps 5 and 7 add
> `maps` and `rviz` blocks to it.

**Why this matters:** if a folder is not listed here, ROS cannot find it at run
time, even though the file is clearly sitting on your disk.

#### `unitree_go2_navigation/config/pointcloud_to_laserscan.yaml`

This is the 3D-to-2D squasher. These are the values **as first written**; step 4
corrects two of them.

```yaml
pointcloud_to_laserscan:
  ros__parameters:
    use_sim_time: true

    # Flatten into the gravity-aligned ground frame, not the sensor frame: the L1
    # is mounted at roll 165 deg / yaw 90 deg, so a height band only means
    # "above the ground" after this transform.
    target_frame: base_footprint
    transform_tolerance: 0.1

    # Band above the ground plane. The lidar sits ~0.19 m up, so 0.1 m clears
    # ground returns from the shallow downward beams while staying under the
    # 1 m tall obstacles in the world.
    min_height: 0.1
    max_height: 0.8

    # Matches the sensor's 600 samples over a full revolution.
    angle_min: -3.14159
    angle_max: 3.14159
    angle_increment: 0.01047
    scan_time: 0.1

    # The sensor's own limits, from lidar_4D_lidar.xacro.
    range_min: 0.8
    range_max: 30.0

    use_inf: true
    inf_epsilon: 1.0
    queue_size: 10
```

Line by line, and **why** each value:

- `use_sim_time: true` — use the simulator's clock, not the wall clock. Nearly
  every mystery failure in ROS simulation traces back to getting this wrong.
- `target_frame: base_footprint` — **this is the critical one.** The lidar is
  bolted on at roll ≈ 165°, nearly upside down. If we filtered by height in the
  sensor's own frame, "up" would not mean up. Converting to `base_footprint`
  first — the flattened, gravity-aligned frame on the floor — makes "height"
  mean real height above the floor.
- `min_height` / `max_height` — keep only points inside this height band. Points
  on the floor are discarded; points above head height are discarded; what is
  left is wall-height obstacles.
- `angle_increment: 0.01047` — 2π ÷ 600 ≈ 0.01047. The lidar takes 600 samples
  per full turn, so this matches the hardware exactly.
- `range_min: 0.8`, `range_max: 30.0` — copied from the lidar's own definition in
  `unitree_go2_description/urdf/lidar_4D_lidar.xacro`. The sensor genuinely
  cannot see closer than 0.8 m. See [section 6](#6-what-is-verified-and-what-is-not).
- `use_inf: true` — report "nothing there" as infinity rather than a fake
  maximum reading, so empty space is not mistaken for a distant wall.

#### `unitree_go2_navigation/config/slam_toolbox.yaml`

The map-builder's settings. This is the stock `slam_toolbox` async
configuration with three deliberate changes, flagged below.

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: true

    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_map_saver: true
    mode: mapping

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    resolution: 0.05
    restamp_tf: false
    # Matches the L1's own limits, so rastering does not invent returns the
    # sensor cannot produce.
    min_laser_range: 0.8
    max_laser_range: 30.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    # General Parameters
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    check_min_dist_and_heading_precisely: false
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation Parameters - Correlation Parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Correlation Parameters - Loop Closure Parameters
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan Matcher Parameters
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0

    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
    min_pass_through: 2
    occupancy_threshold: 0.1
```

The settings that matter, and everything else left at stock on purpose:

- `use_sim_time: true` — **changed from stock.** Simulator clock.
- `min_laser_range: 0.8` / `max_laser_range: 30.0` — **changed from stock**
  (`0.0` / `20.0`). Matched to this lidar so the map does not claim readings the
  sensor cannot make.
- `base_frame: base_footprint` — must match `target_frame` in the file above.
- `resolution: 0.05` — each map square is 5 cm.
- `minimum_travel_distance: 0.5` — only record a new scan after moving 0.5 m,
  which stops the map filling with duplicates while standing still.
- `mode: mapping` — build a map, as opposed to re-using one.
- Everything else is the stock file, left alone deliberately. Tuning values you
  do not understand is how SLAM gets broken.

#### `unitree_go2_navigation/launch/slam.launch.py`

A **launch file** starts several programs at once with the right settings.

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    unitree_go2_sim = get_package_share_directory("unitree_go2_sim")
    unitree_go2_navigation = get_package_share_directory("unitree_go2_navigation")

    pointcloud_to_laserscan_config = os.path.join(
        unitree_go2_navigation, "config", "pointcloud_to_laserscan.yaml"
    )
    slam_toolbox_config = os.path.join(
        unitree_go2_navigation, "config", "slam_toolbox.yaml"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )

    # slam_toolbox owns map->odom here, so the sim must not also publish it.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unitree_go2_sim, "launch", "unitree_go2_launch.py")
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "disable_static_map_odom": "true",
        }.items(),
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[pointcloud_to_laserscan_config],
        remappings=[
            ("cloud_in", "/unitree_lidar/points"),
            ("scan", "/scan"),
        ],
    )

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_toolbox_config, {"use_lifecycle_manager": False}],
    )

    # slam_toolbox is a lifecycle node and stays unconfigured until driven
    # through these transitions.
    configure_slam_toolbox = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_slam_toolbox = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            declare_gui,
            declare_rviz,
            sim,
            pointcloud_to_laserscan_node,
            slam_toolbox_node,
            configure_slam_toolbox,
            activate_slam_toolbox,
        ]
    )
```

Section by section:

- `get_package_share_directory(...)` finds where a package was installed, so no
  file paths are hard-coded.
- `IncludeLaunchDescription(...)` runs the simulator's own launch file, passing
  `disable_static_map_odom: "true"` — the switch from step 2. This is where
  SLAM takes ownership of `map` → `odom`.
- `remappings` renames channels **without editing code**. The
  `pointcloud_to_laserscan` program publishes on a channel it calls `scan` and
  listens on one it calls `cloud_in`; we point those at this robot's real names,
  `/unitree_lidar/points` and `/scan`.
- `LifecycleNode` — not a normal `Node`. `slam_toolbox` on ROS 2 Jazzy starts
  **asleep** and does nothing until told to wake up.
- `configure_slam_toolbox` and `activate_slam_toolbox` are the wake-up calls:
  first *configure*, then once that finishes, *activate*. Without these two
  blocks, `slam_toolbox` sits there doing nothing and produces no map at all.
  This caught us out — see [Problem 1](#problem-1-slam_toolbox-does-nothing).

**Try it:**

```bash
cd ~/ws && colcon build --packages-select unitree_go2_navigation
source install/setup.bash
ros2 launch unitree_go2_navigation slam.launch.py gui:=false rviz:=false
```

Wait **60–90 seconds**. In another terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ws/install/setup.bash
ros2 lifecycle get /slam_toolbox
ros2 topic list | grep -E "^/scan$|^/map$"
```

**What you should see:**

```
active [3]
/map
/scan
```

`active [3]` is the critical one — it means `slam_toolbox` is awake and working.

**If it does not work:**

- `unconfigured [1]` → the two wake-up blocks are missing or did not fire.
- `/scan` missing → check the `remappings`. Also note `ros2 topic echo /scan`
  prints nothing even when it is working, because of a settings mismatch — see
  [Problem 8](#problem-8-ros2-topic-echo-scan-shows-nothing).

---

### Step 4 — Lift the height band clear of the floor

**Commit:** `6cee848` — *fix(navigation): raise the scan height band clear of the ground*

**What it is for:** With the first values, the robot sometimes saw the **floor**
as a wall. The fix is to raise the bottom of the band.

**File changed:** `unitree_go2_navigation/config/pointcloud_to_laserscan.yaml`

```diff
-    # Band above the ground plane. The lidar sits ~0.19 m up, so 0.1 m clears
-    # ground returns from the shallow downward beams while staying under the
-    # 1 m tall obstacles in the world.
-    min_height: 0.1
-    max_height: 0.8
+    # Band above the ground plane. Measured in this frame the ground sits at
+    # about -0.05 m, so 0.25 m leaves margin for the body pitching as it walks
+    # before ground returns would register as obstacles.
+    min_height: 0.25
+    max_height: 1.1
```

**Why these numbers.** We measured the actual point cloud, converted into
`base_footprint`, with the robot standing still:

- The floor appeared at about **−0.05 m**, not exactly 0.
- The obstacles in this world are **1 m tall**.
- A dog walks by rocking its body. That rocking tilts the lidar, which swings
  distant floor points up and down by tens of centimetres.

`0.1 m` left only 15 cm of margin above the floor — not enough once the body
rocks. `0.25 m` roughly doubles the margin. `1.1 m` was raised from `0.8 m` so
the **full height** of a 1 m obstacle is captured, giving more points to match
against.

With the robot upright, the band kept **333 of 18,000** points per scan, and all
of them were at 3.6–5.3 m — exactly where the real obstacles are, and nothing
from the floor.

**Try it:**

```bash
cd ~/ws && colcon build --packages-select unitree_go2_navigation && source install/setup.bash
ros2 launch unitree_go2_navigation slam.launch.py gui:=false rviz:=false
```

**What you should see:** after the robot drives, the map shows a handful of
separate blobs, not a solid ring around the robot.

**If it does not work:** a solid ring of obstacles at roughly 1–2 m in every
direction means the floor is being read as a wall. Nine times out of ten the
robot has fallen over — check with
[Problem 2](#problem-2-the-robot-falls-over).

---

### Step 5 — Save a map of the world into the repository

**Commit:** `b7d44d6` — *feat(navigation): add a map of the default world*

**What it is for:** Localisation needs a map that already exists. This saves one
into the repository so the demo works out of the box.

**Files created:** `unitree_go2_navigation/maps/default_world.pgm`,
`unitree_go2_navigation/maps/default_world.yaml`
**File changed:** `unitree_go2_navigation/CMakeLists.txt`

```diff
 install(
   DIRECTORY config
   DESTINATION share/${PROJECT_NAME}
 )
+
+install(
+  DIRECTORY maps
+  DESTINATION share/${PROJECT_NAME}
+)
```

To produce the map yourself: start mapping, drive the robot around, then save.

```bash
# terminal 1
ros2 launch unitree_go2_navigation slam.launch.py gui:=false rviz:=false
```

Wait **60–90 seconds**, then drive it in terminal 2:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> **Drive gently.** Tap the keys; do not hold them. Sudden changes make the dog
> trip and fall, which silently ruins the map. See
> [Problem 2](#problem-2-the-robot-falls-over).

Then save, in terminal 3:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ws/install/setup.bash
mkdir -p ~/ws/src/unitree_go2_ros2/unitree_go2_navigation/maps
cd ~/ws/src/unitree_go2_ros2/unitree_go2_navigation/maps
ros2 run nav2_map_server map_saver_cli -f default_world
```

**What you should see:**

```
[INFO] [map_io]: Map saved
[INFO] [map_saver]: Map saved successfully
```

and two new files, `default_world.pgm` and `default_world.yaml`.

The `.yaml` file is small and readable:

```yaml
image: default_world.pgm
mode: trinary
resolution: 0.050
origin: [-28.010, -26.008, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

- `image` — the picture file holding the map itself.
- `resolution: 0.050` — each pixel is 5 cm.
- `origin` — where the bottom-left corner of the picture sits in the world.
- `occupied_thresh` / `free_thresh` — how dark a pixel must be to count as
  blocked, and how light to count as free. Anything between the two is
  "unknown".

**If it does not work:** `Failed to spin map subscription` almost always means
there is no map to save — see [Problem 3](#problem-3-map_saver_cli-fails).

> **Checking your map is any good.** The world contains five obstacles, at
> (5, 0), (−3, 4), (0, −5), (3, 3) and (−4, −3). A good map shows five separate
> blobs near those positions. The map committed here has 423 blocked cells
> forming those five blobs, plus one small 31-cell artefact at about
> (2.98, −4.51) that does not correspond to anything real.
>
> Note that each blob sits roughly 0.5 m *toward* the robot from the true
> centre. That is correct and expected: the lidar only ever sees the **near
> face** of an object, never through it.

---

### Step 6 — Add Nav2

**Commit:** `6df80a3` — *feat(navigation): add Nav2 bringup for mapping and localization*

**What it is for:** This is the actual autonomy. It adds Nav2's settings and a
launch file offering both modes: build-a-map-while-driving, or
drive-on-a-saved-map.

**Files created:** `unitree_go2_navigation/config/nav2_params.yaml` (428 lines),
`unitree_go2_navigation/launch/nav2.launch.py`

#### Getting `nav2_params.yaml`

Start from Nav2's own example file, then edit it:

```bash
cp /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml \
   ~/ws/src/unitree_go2_ros2/unitree_go2_navigation/config/nav2_params.yaml
```

Now the edits that were made, grouped by what they do. You can see the final
values with `grep`, shown under each group.

**(a) Where the robot is, and how big it is**

```yaml
      robot_base_frame: base_footprint   # was base_link, in several places
      robot_radius: 0.35                 # was 0.22
```

`base_footprint` is the flattened on-the-floor frame, and it is what both
`slam_toolbox` and AMCL use — so the costmaps must use it too or the robot's
position would be inconsistent between them.

`robot_radius: 0.35` is the radius of a circle that contains the whole dog. The
body alone is 0.376 × 0.094 m, but with legs the standing footprint is roughly
0.65 × 0.35 m, so 0.35 m is the circle that wraps it. The stock 0.22 was for a
much smaller robot.

**(b) Seed AMCL's starting position**

```yaml
    scan_topic: scan

    # The robot always spawns at the map origin in this world, so seed AMCL
    # there instead of requiring a manual "2D Pose Estimate" in RViz.
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
```

Without this you must click "2D Pose Estimate" in RViz every single run before
anything works. Since the robot always starts at the same place, we just tell
AMCL where that is.

**(c) The local costmap — what is around me right now**

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.35
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.70
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          ...
          raytrace_max_range: 5.0
          obstacle_max_range: 4.0
```

- `rolling_window: true`, `width`/`height: 6` — a 6 × 6 m square that travels
  with the robot. Raised from the stock 3 × 3 m because this lidar sees far
  further than the small robot the stock file was written for.
- `obstacle_layer` — **changed from stock**, which used a 3D `VoxelLayer`. We
  only have a flat 2D scan, so a 3D layer wastes processing power for nothing.
- `inflation_radius: 0.70` — draw a 70 cm "keep out" halo around every obstacle.
  It must be larger than `robot_radius` (0.35) or the planner will try to squeeze
  the robot through gaps it cannot fit.

**(d) The global costmap — the whole known world**

```yaml
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

- `static_layer` — the saved map.
- `obstacle_layer` — things the lidar sees now that are not on the saved map.
- `inflation_layer` — the same keep-out halo.

**(e) The route planner**

```yaml
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

`NavFn` finds the shortest route across the map. It is left at stock. The dog
can turn on the spot, so it does not need the more complicated car-style
planner. `allow_unknown: true` lets it plan through unexplored space.

**(f) The route follower — and why DWB, not MPPI**

```yaml
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.26
      max_vel_theta: 0.45
      acc_lim_x: 0.5
      acc_lim_theta: 1.0
      decel_lim_x: -0.7
      decel_lim_theta: -1.4
      vx_samples: 20
      vtheta_samples: 20
      sim_time: 2.0
      xy_goal_tolerance: 0.25
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign",
                "PathAlign", "PathDist", "GoalDist"]
```

The stock file uses a follower called **MPPI**. We measured it failing on this
machine: it could only run at about **5.5 updates per second** instead of the 20
it needs, and commanded only about **0.06 m/s** — so slow that Nav2 decided the
robot was stuck and gave up. **DWB** is simpler and cheaper, and worked.

- `max_vel_x: 0.26`, `max_vel_theta: 0.45` — just under the dog's real limits
  (0.3 m/s and 0.5 rad/s) from
  `unitree_go2_sim/config/gait/gait.yaml`. Going over makes it fall.
- `max_vel_y: 0.0` — the dog *can* step sideways, but we forbid it. Keeping it
  to forward-and-turn makes the gait much more stable.
- `acc_lim_x: 0.5` — how fast speed may change. Deliberately gentle; this is the
  main protection against tipping over.
- `sim_time: 2.0` — imagine 2 seconds into the future when judging each
  candidate move.
- `critics` — the scorecard for candidate moves: stay near the path
  (`PathDist`, `PathAlign`), head toward the goal (`GoalDist`, `GoalAlign`),
  avoid obstacles (`BaseObstacle`), do not dither (`Oscillation`), turn to face
  the right way at the end (`RotateToGoal`).

**(g) The speed limiter**

```yaml
velocity_smoother:
  ros__parameters:
    max_velocity: [0.30, 0.25, 0.50]
    min_velocity: [-0.20, -0.25, -0.50]
    max_accel: [0.40, 0.40, 0.80]
    max_decel: [-0.60, -0.60, -1.20]
```

Each list is `[forward, sideways, turning]`. The speeds are the dog's real
limits from `gait.yaml`. The accelerations are deliberately low so commands ramp
instead of jumping. The stock values were `[2.5, 0.0, 3.2]` — more than six
times harsher, and enough to put the dog on its back.

**(h) The final safety gate**

```yaml
collision_monitor:
  ros__parameters:
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
```

This is what makes `collision_monitor` the **last** link in the chain and the
only program publishing `/cmd_vel`.

#### `unitree_go2_navigation/launch/nav2.launch.py`

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    unitree_go2_sim = get_package_share_directory("unitree_go2_sim")
    unitree_go2_navigation = get_package_share_directory("unitree_go2_navigation")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    pointcloud_to_laserscan_config = os.path.join(
        unitree_go2_navigation, "config", "pointcloud_to_laserscan.yaml"
    )
    slam_toolbox_config = os.path.join(
        unitree_go2_navigation, "config", "slam_toolbox.yaml"
    )
    nav2_params = os.path.join(unitree_go2_navigation, "config", "nav2_params.yaml")
    default_map = os.path.join(unitree_go2_navigation, "maps", "default_world.yaml")

    mode = LaunchConfiguration("mode")
    is_slam = PythonExpression(["'", mode, "' == 'slam'"])

    declare_mode = DeclareLaunchArgument(
        "mode",
        default_value="slam",
        description="'slam' builds a map with slam_toolbox, "
        "'localization' localises on an existing map with AMCL",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Map yaml to localise against (mode:=localization only)",
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use the Gazebo gui"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    declare_nav2_delay = DeclareLaunchArgument(
        "nav2_delay",
        default_value="25.0",
        description="Seconds to wait for Gazebo to spawn the robot before "
        "bringing up Nav2, so the costmaps find a complete TF tree",
    )

    # Whatever provides map->odom (slam_toolbox or AMCL) owns it, so the sim
    # must not publish its static identity transform as well.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unitree_go2_sim, "launch", "unitree_go2_launch.py")
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "disable_static_map_odom": "true",
        }.items(),
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[pointcloud_to_laserscan_config],
        remappings=[
            ("cloud_in", "/unitree_lidar/points"),
            ("scan", "/scan"),
        ],
    )

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_toolbox_config, {"use_lifecycle_manager": False}],
        condition=IfCondition(is_slam),
    )

    # slam_toolbox is a lifecycle node and stays unconfigured until driven
    # through these transitions.
    configure_slam_toolbox = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(is_slam),
    )

    activate_slam_toolbox = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(is_slam),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": "true",
            "params_file": nav2_params,
        }.items(),
        condition=UnlessCondition(is_slam),
    )

    # Nav2 is brought up node by node rather than via nav2_bringup's
    # navigation_launch.py, which also starts docking_server and route_server.
    # docking_server publishes /cmd_vel, which would give the robot a second
    # command publisher; this robot must have exactly one.
    #
    # Velocity path: controller_server -> cmd_vel_nav -> velocity_smoother
    # -> cmd_vel_smoothed -> collision_monitor -> cmd_vel. Only
    # collision_monitor publishes /cmd_vel, which is what CHAMP's
    # quadruped_controller_node consumes.
    nav2_common = {
        "parameters": [nav2_params, {"use_sim_time": True}],
        "output": "screen",
    }
    to_cmd_vel_nav = [("cmd_vel", "cmd_vel_nav")]

    nav2_nodes = [
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", remappings=to_cmd_vel_nav, **nav2_common),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", **nav2_common),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", remappings=to_cmd_vel_nav, **nav2_common),
        Node(package="nav2_smoother", executable="smoother_server",
             name="smoother_server", **nav2_common),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", **nav2_common),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", remappings=to_cmd_vel_nav, **nav2_common),
        Node(package="nav2_collision_monitor", executable="collision_monitor",
             name="collision_monitor", **nav2_common),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"autostart": True},
                {
                    "node_names": [
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "velocity_smoother",
                        "collision_monitor",
                    ]
                },
            ],
        ),
    ]

    return LaunchDescription(
        [
            declare_mode,
            declare_map,
            declare_gui,
            declare_rviz,
            declare_nav2_delay,
            sim,
            pointcloud_to_laserscan_node,
            slam_toolbox_node,
            configure_slam_toolbox,
            activate_slam_toolbox,
            TimerAction(
                period=LaunchConfiguration("nav2_delay"),
                actions=[localization] + nav2_nodes,
            ),
        ]
    )
```

The parts that matter:

- `is_slam` is a yes/no test on the `mode` argument. `IfCondition(is_slam)`
  starts `slam_toolbox` only in SLAM mode; `UnlessCondition(is_slam)` starts
  AMCL only in the other mode. This is how one file serves both, while
  guaranteeing only one of them ever owns `map` → `odom`.
- `TimerAction(period=...nav2_delay...)` delays Nav2 by **40 seconds**. Gazebo
  needs time to drop the robot into the world *and* finish loading its
  controllers. Start Nav2 too early and two things go wrong: its costmaps look
  for a TF chain that does not exist yet and never recover, and the CPU
  contention makes Nav2's own start-up service calls time out
  ([Problem 12](#problem-12-failed-to-bring-up-all-requested-nodes)).
- **Nav2 is started node by node, on purpose.** The obvious thing would be to
  include Nav2's own `navigation_launch.py`. We did that first, and it also
  started `docking_server`, which publishes `/cmd_vel` — giving the robot **two**
  programs issuing commands. Listing the seven nodes by hand excludes it.
- `to_cmd_vel_nav` redirects the planner's output away from `/cmd_vel` and into
  `/cmd_vel_nav`, so it must pass through the smoother and the collision monitor
  before reaching the robot.
- `lifecycle_manager` with `autostart: True` wakes all seven Nav2 nodes, the
  same way the two blocks earlier wake `slam_toolbox`.

**Try it:**

```bash
cd ~/ws && colcon build --packages-select unitree_go2_navigation && source install/setup.bash
ros2 launch unitree_go2_navigation nav2.launch.py mode:=localization gui:=false rviz:=false
```

Wait **60–90 seconds**, then in another terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ws/install/setup.bash
ros2 topic info /cmd_vel | grep -i "publisher count"
ros2 lifecycle get /bt_navigator
```

**What you should see:**

```
Publisher count: 1
active [3]
```

`Publisher count: 1` is the one to care about. If it says `2`, something else is
also commanding the robot.

Now send it somewhere:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

**What you should see:** the dog walks forward, and the command finishes with
`Goal finished with status: SUCCEEDED`.

**If it does not work:**

- `Publisher count: 2` → see [Problem 9](#problem-9-two-programs-command-the-robot).
- `bt_navigator` not `active`, with `Invalid frame ID "odom"` in the log → the
  TF chain was missing when Nav2 started. See
  [Problem 6](#problem-6-rviz-breaks-the-whole-stack).
- The robot wanders away and the log says `Failed to make progress` → the
  follower is too slow. See [Problem 10](#problem-10-the-robot-wanders-off).

---

### Step 7 — One command for the whole demo

**Commit:** `bc4b6a4` — *feat(navigation): add a one-shot demo launch with a Nav2 RViz view*

**What it is for:** A single command that starts everything including a
pre-configured RViz view, so a demo needs no setup.

**Files created:** `unitree_go2_navigation/launch/demo_nav.launch.py`,
`unitree_go2_navigation/rviz/nav2_go2.rviz` (603 lines)
**File changed:** `unitree_go2_navigation/CMakeLists.txt`

Get the RViz view file by copying Nav2's and pointing the camera at the dog's
frame:

```bash
mkdir -p ~/ws/src/unitree_go2_ros2/unitree_go2_navigation/rviz
cp /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
   ~/ws/src/unitree_go2_ros2/unitree_go2_navigation/rviz/nav2_go2.rviz
sed -i 's/Target Frame: base_link/Target Frame: base_footprint/' \
   ~/ws/src/unitree_go2_ros2/unitree_go2_navigation/rviz/nav2_go2.rviz
```

And add it to `CMakeLists.txt`:

```diff
 install(
   DIRECTORY maps
   DESTINATION share/${PROJECT_NAME}
 )
+
+install(
+  DIRECTORY rviz
+  DESTINATION share/${PROJECT_NAME}
+)
```

#### `unitree_go2_navigation/launch/demo_nav.launch.py`

```python
"""One-shot demo: sim + mapping-or-localization + Nav2 + a Nav2 RViz view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    unitree_go2_navigation = get_package_share_directory("unitree_go2_navigation")

    default_map = os.path.join(unitree_go2_navigation, "maps", "default_world.yaml")
    rviz_config = os.path.join(unitree_go2_navigation, "rviz", "nav2_go2.rviz")

    mode = LaunchConfiguration("mode")
    odom_source = LaunchConfiguration("odom_source")

    declare_mode = DeclareLaunchArgument(
        "mode",
        default_value="localization",
        description="'localization' navigates on the saved map with AMCL, "
        "'slam' maps and navigates at the same time with slam_toolbox",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Map yaml to localise against (mode:=localization only)",
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use the Gazebo gui"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz with the Nav2 view"
    )
    declare_rviz_delay = DeclareLaunchArgument(
        "rviz_delay",
        default_value="45.0",
        description="Seconds before RViz starts. Starting RViz at the same "
        "time as the rest of the stack saturates DDS discovery here and "
        "/clock stops being delivered, which blocks Nav2 activation; "
        "bringing it up once everything else is settled avoids that.",
    )
    declare_nav2_delay = DeclareLaunchArgument(
        "nav2_delay",
        default_value="25.0",
        description="Seconds to wait for Gazebo to spawn the robot before Nav2 starts",
    )
    # Stub for a later phase. Nav2 consumes odom->base_footprint without caring
    # who publishes it, so swapping in FAST-LIO/Point-LIO should not require any
    # change to nav2_params.yaml.
    declare_odom_source = DeclareLaunchArgument(
        "odom_source",
        default_value="ekf",
        description="Source of odom->base_footprint: 'ekf' (robot_localization, "
        "the only one implemented) or 'lio' (reserved for FAST-LIO/Point-LIO)",
    )

    warn_lio = LogInfo(
        msg=[
            "[demo_nav] odom_source:='",
            odom_source,
            "' is not implemented yet; falling back to the robot_localization "
            "EKFs. Nav2 configuration is unchanged either way.",
        ],
        condition=IfCondition(PythonExpression(["'", odom_source, "' != 'ekf'"])),
    )

    # nav2.launch.py brings up the sim, the scan conversion, slam_toolbox or
    # AMCL, and Nav2. RViz is started here instead so the demo gets the Nav2
    # view rather than the plain simulation one.
    # Scoped: nav2.launch.py also declares a 'rviz' argument, and passing
    # rviz:=false into it would otherwise leak back out and suppress the Nav2
    # RViz view started below.
    nav2 = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unitree_go2_navigation, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "mode": mode,
            "map": LaunchConfiguration("map"),
            "gui": LaunchConfiguration("gui"),
            "rviz": "false",
            "nav2_delay": LaunchConfiguration("nav2_delay"),
        }.items(),
    )],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            declare_mode,
            declare_map,
            declare_gui,
            declare_rviz,
            declare_nav2_delay,
            declare_rviz_delay,
            declare_odom_source,
            warn_lio,
            nav2,
            TimerAction(
                period=LaunchConfiguration("rviz_delay"),
                actions=[rviz_node],
            ),
        ]
    )
```

Three parts deserve explanation:

- **`mode` defaults to `localization` here**, the opposite of `nav2.launch.py`.
  For a demo you normally want the saved map, not to rebuild one.
- **`GroupAction(scoped=True, ...)`** — a fence. This file passes `rviz: "false"`
  into `nav2.launch.py` so the *simulator's* plain RViz does not start. Without
  the fence that `false` leaked back out and switched off **this** file's RViz
  too, so no RViz appeared at all. The fence keeps the setting inside.
- **`rviz` defaults to `false`** — RViz is *not* started by this file. Starting
  it from the launch file broke the whole stack, and delaying it only made the
  failure intermittent rather than fixing it. `rviz_delay` (45 s) applies only
  if you force `rviz:=true`. See
  [Problem 6](#problem-6-rviz-breaks-the-whole-stack).
- **`odom_source`** — an argument that currently does nothing except print a
  warning if set to `lio`. It marks where a future laser-based odometry system
  will plug in. **`lio` is not implemented.**

**Try it:**

```bash
cd ~/ws && colcon build --packages-select unitree_go2_navigation && source install/setup.bash
ros2 launch unitree_go2_navigation demo_nav.launch.py
```

**What you should see:** a Gazebo window with the dog. **No RViz** — it is off
by default, for the reason in
[Problem 6](#problem-6-rviz-breaks-the-whole-stack). Start it by hand once the
stack is up (section 5).

**If it does not work:** everything frozen with Nav2 never going `active` → see
[Problem 6](#problem-6-rviz-breaks-the-whole-stack).

---

### Step 8 — Write it down

**Commit:** `74cdcf2` — *docs: document mapping, navigation and the demo*

**File changed:** `README.md` — adds a quick-start section, the TF ownership
table, the `/cmd_vel` chain, the startup wait, and the known limitations.

No command to run. Documentation that does not exist is documentation nobody
follows.

---

## 4. Problems we hit, and the fixes

These all actually happened while building this.

### Problem 1 — `slam_toolbox` does nothing

**Symptom:** It starts without errors, but no `/map` ever appears. Asking it for
settings returns `Parameter not set`, and `/scan` shows zero subscribers.

**Cause:** On ROS 2 Jazzy, `slam_toolbox` is a **lifecycle node**. It starts in
an `unconfigured` state and waits to be told to wake up. It is not broken and it
is not misconfigured — it is asleep.

**Fix:** Drive it through *configure* then *activate* in the launch file, using
`EmitEvent` and `RegisterEventHandler` (step 3). Check with:

```bash
ros2 lifecycle get /slam_toolbox
```

`active [3]` is correct. `unconfigured [1]` means the wake-up never happened.

---

### Problem 2 — The robot falls over

**Symptom:** The dog ends up on its back. The map fills with a solid ring of
obstacles 1–2 m away in every direction.

**Cause:** Sudden changes on `/cmd_vel`. The gait cannot absorb a step change
from stopped to full speed, so it trips. Once it is lying down the lidar points
sideways, the floor lands inside the height band, and every direction looks
blocked.

**Fix:** Ramp speeds up and down instead of stepping. When driving by keyboard,
tap keys rather than holding them. Nav2 does this for you via the
`velocity_smoother` acceleration limits in `nav2_params.yaml`.

Check whether the dog is upright — `z` should be about **0.24**, not 0.1:

```bash
ros2 topic echo --once --field pose.pose.position /odom/ground_truth
```

> Any scan data recorded while it was down is rubbish. Restart and remap.

---

### Problem 3 — `map_saver_cli` fails

**Symptom:**

```
[ERROR] [map_saver]: Failed to spin map subscription
[ros2run]: Process exited with failure 1
```

**Cause:** The message blames the subscription, which sends you hunting for a
settings mismatch. The real cause is almost always that **there is no map to
save** — either `slam_toolbox` is asleep (Problem 1), or you are running the
plain simulator launch file, which contains no SLAM at all.

**Fix:** Confirm a map exists before saving:

```bash
ros2 topic list | grep /map
ros2 lifecycle get /slam_toolbox
```

You need `/map` listed and `active [3]`. Note that raising the save timeout does
**not** help, which is a useful clue that the timeout is not the problem.

---

### Problem 4 — Two simulators at once

**Symptom:** A flood of these, and the dog collapses:

```
[tf2_buffer]: Detected jump back in time. Clearing TF buffer.
[rviz2]: Detected jump back in time. Resetting RViz.
[controller_manager]: Caught exception ... Pid is called with negative dt
[controller_manager]: Deactivating controllers : [ joint_group_effort_controller ]
```

**Cause:** Two Gazebo instances running at once. Both publish the time, at
different values, so time appears to jump backwards. The leg controller divides
by a negative time step, throws an error, and shuts itself down.

**Fix:** Only ever one simulator. Before launching, check:

```bash
pgrep -af "gz sim"
```

Empty output means you are clear. To clean up thoroughly:

```bash
for p in "[r]os2 launch" "[g]z sim" "[p]arameter_bridge" "[e]kf_node" \
         "[r]obot_state_publisher" "[s]lam_toolbox" "[p]ointcloud_to_laserscan" \
         "[r]viz2" "[a]mcl" "[m]ap_server" "[c]ontroller_server"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 4
ros2 daemon stop
pgrep -af "[g]z sim" || echo "all clear"
```

> **Why the odd `[g]z` spelling?** Writing `pkill -f "gz sim"` matches the
> `pkill` command's *own* command line, so it kills its own shell before
> finishing the job. The square brackets stop it matching itself. This cost us
> real time: processes we thought we had killed were still running.

---

### Problem 5 — Things look dead but are actually running

**Symptom:** `ros2 node list` shows nothing, or shows nodes that no longer
exist.

**Cause:** The ROS command-line tools keep a background helper that caches what
it has seen. After restarting the simulator, that cache is stale.

**Fix:**

```bash
ros2 daemon stop
```

It restarts automatically on the next command. Do this after every restart,
before trusting what the tools tell you.

---

### Problem 6 — RViz breaks the whole stack

**Symptom:** With RViz started at the same time as everything else: Gazebo runs
and its own clock advances, but `/clock` goes silent on the ROS side, the whole
TF chain disappears, and Nav2 never becomes `active`. The logs fill with:

```
[local_costmap]: Timed out waiting for transform from base_footprint to odom ...
  Invalid frame ID "odom" passed to canTransform argument target_frame
```

**Cause:** Too many programs appearing at once saturates DDS discovery — the
mechanism ROS programs use to find each other. We measured **41 subscribers** on
`/clock`. Gazebo was publishing it perfectly; nothing on the ROS side received
it.

Confirmed by elimination: identical launch with `rviz:=false` worked every time;
starting RViz **by hand after** the stack had settled also worked every time.

**Fix:** Do not let the launch file start RViz. `demo_nav.launch.py` has
`rviz:=false` by default. Bring the stack up, confirm it is ready, and only then
start RViz by hand in another terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ws/install/setup.bash
rviz2 -d ~/ws/install/unitree_go2_navigation/share/unitree_go2_navigation/rviz/nav2_go2.rviz \
  --ros-args -p use_sim_time:=true
```

> We first tried delaying RViz by 45 seconds inside the launch file. That worked
> once and then failed on the next run with identical timing, so **the delay is
> not a reliable fix** and is no longer the default. Starting RViz by hand after
> the stack is confirmed up worked every time we tried it.
>
> A proper fix would mean tuning DDS settings in the container image, which was
> out of scope. If the stack has already lost `/clock`, it does **not** recover
> when you close RViz — you must restart it.

---

### Problem 7 — Missing packages after a container restart

**Symptom:**

```
package 'realsense2_description' not found
package 'pointcloud_to_laserscan' not found
```

**Cause:** The container runs with `--rm`, so it is deleted when it stops.
Anything installed by hand inside it disappears with it.

**Fix:** Put packages in the image, not in the running container. They belong in
`~/go2_docker/Dockerfile`, which already lists:

```
    ros-jazzy-realsense2-description \
    ros-jazzy-pointcloud-to-laserscan \
```

Rebuild the image after changing it:

```bash
~/go2_docker/run.sh build
```

Or, from inside the container, let ROS work out what is missing from the
`package.xml` files:

```bash
cd ~/ws
rosdep install --from-paths src --ignore-src -y
```

---

### Problem 8 — `ros2 topic echo /scan` shows nothing

**Symptom:** `/scan` is listed and other programs clearly use it, but `echo`
prints nothing and `hz` says "does not appear to be published".

**Cause:** Sensor topics are published "best effort" (drop messages rather than
wait). The command-line tool asks for "reliable" by default. The two do not
match, so no messages are delivered.

**Fix:** Ask for the matching setting:

```bash
ros2 topic echo /scan --qos-reliability best_effort --once
```

This is a **reporting** artefact, not a fault. We briefly chased it as a real
failure.

---

### Problem 9 — Two programs command the robot

**Symptom:**

```bash
ros2 topic info /cmd_vel | grep -i "publisher count"
# Publisher count: 2
```

**Cause:** Nav2's own `navigation_launch.py` starts `docking_server` as well,
and that publishes `/cmd_vel`. Two programs commanding the same robot is a
recipe for unpredictable behaviour.

**Fix:** Start the seven Nav2 nodes by hand and leave `docking_server` out
(step 6). Verify:

```bash
ros2 topic info /cmd_vel --verbose | grep -A1 "Publisher count"
```

Exactly one publisher, and it should be `collision_monitor`.

---

### Problem 10 — The robot wanders off instead of reaching the goal

**Symptom:** Distance to the goal *grows*. The log repeats:

```
[controller_server]: Failed to make progress
[behavior_server]: Running spin
```

**Cause:** Not a wrong route — those were Nav2's recovery behaviours. The
follower (MPPI) was commanding only about **0.06 m/s**, below the threshold Nav2
uses to decide the robot is stuck. MPPI was managing about **5.5 updates per
second** against the 20 it expects.

We proved the robot itself was fine by commanding it directly: `0.25 m/s`
forward for 6 s moved it exactly 1.50 m, and `0.40 rad/s` turned it 151°. The
dog was obeying perfectly.

**Fix:** Switch the follower to **DWB** (step 6f), which is much cheaper to run.

> This is a judgement about **this machine under this load**. MPPI may well be
> the better choice on a less loaded computer.

---

### Problem 11 — Nav2 starts before the robot exists

**Symptom:** Costmaps complain `Invalid frame ID "odom"` forever, even though
the simulator eventually comes up fine.

**Cause:** Gazebo takes roughly 30–40 seconds to place the robot and start
publishing positions. Nav2 starting first finds nothing and does not recover.

**Fix:** The 40-second `nav2_delay` in `nav2.launch.py`, and **waiting 90–120
seconds** before testing anything. Check readiness rather than guessing:

```bash
ros2 topic hz /clock
ros2 run tf2_ros tf2_echo map base_footprint
ros2 lifecycle get /bt_navigator
```

You want the clock ticking, the transform printing numbers, and `active [3]`.

---

### Problem 12 — "Failed to bring up all requested nodes"

**Symptom:** Nav2 never starts. Asking any Nav2 node for its state returns
`unconfigured [1]`, `/cmd_vel` has zero publishers, and the log ends with:

```
[lifecycle_manager_navigation]: Failed to change state for node: smoother_server.
  Exception: smoother_server/get_state service client: async_send_request failed.
[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

**Cause:** Not a broken configuration. Looking at `smoother_server`'s own log, it
configured perfectly:

```
[smoother_server]: Configuring smoother server
[smoother_server]: Smoother Server has simple_smoother  smoothers available.
[smoother_server.rclcpp]: failed to send response to /smoother_server/get_state (timeout)
```

Its **reply** was dropped. Nav2's manager asks each node "are you ready?" and
gives up if the answer does not arrive in time. While Gazebo is still loading
its controllers the machine is busy enough for that reply to be lost, and the
manager then aborts the whole bringup rather than retrying.

**Fix:** Give Gazebo longer to settle before Nav2 starts. `nav2_delay` was
raised from 25 to **40 seconds** for this reason. If it still happens, restart
and increase it further:

```bash
ros2 launch unitree_go2_navigation demo_nav.launch.py nav2_delay:=60.0
```

**It can also stall rather than abort.** If the manager stops at
`Configuring controller_server` and never continues, look at that node's own
log. We saw it finish configuring —

```
[controller_server]: Controller Server has FollowPath  controllers available.
[controller_server.rclcpp]: failed to send response to /controller_server/change_state (timeout)
```

— and the *reply* go missing. The symptom differs; the cause is the same.

**If it happens, restart.** It does not recover on its own.

> **This is a known weakness, not a solved problem.** It is intermittent and
> load-dependent: the same command succeeds on one run and fails on the next
> with identical timing. We measured a full localization run with two
> successful goals at `nav2_delay:=40.0`, and a SLAM run that stalled at the
> same setting.
>
> The underlying cause is the DDS middleware dropping service replies under
> load. There is **no setting in this repository that fixes it** —
> `lifecycle_manager` exposes only `bond_timeout`, which governs a different
> thing. A real fix means changing the container image: either installing and
> switching to CycloneDDS (`rmw_cyclonedds_cpp`, not currently installed) or
> supplying a tuned FastDDS XML profile. Both were out of scope here.
>
> Until then: raise `nav2_delay`, check readiness before sending goals, and
> restart if Nav2 does not reach `active`.

---

## 5. Running the demo from a fresh container

### Start the container and build

Terminal 1, on your computer:

```bash
~/go2_docker/run.sh
```

Then inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ws
colcon build
source install/setup.bash
```

**What you should see:** `Summary: 7 packages finished`. The first build takes
several minutes; later ones take seconds.

### Mode A — navigate on the saved map (the normal demo)

```bash
ros2 launch unitree_go2_navigation demo_nav.launch.py
```

**Wait 90–120 seconds.** A Gazebo window appears. Nav2 itself only starts 40
seconds in, so give it longer than you think. Start-up errors during this window
are normal. RViz is deliberately **not** started here — see below.

Terminal 2:

```bash
~/go2_docker/run.sh shell
source /opt/ros/jazzy/setup.bash && source ~/ws/install/setup.bash
```

Check it is ready before sending anything:

```bash
ros2 lifecycle get /bt_navigator
ros2 topic info /cmd_vel | grep -i "publisher count"
```

**What you should see:** `active [3]` and `Publisher count: 1`.

**Only now**, if you want the visual view, start RViz in a third terminal:

```bash
~/go2_docker/run.sh shell
source /opt/ros/jazzy/setup.bash && source ~/ws/install/setup.bash
rviz2 -d ~/ws/install/unitree_go2_navigation/share/unitree_go2_navigation/rviz/nav2_go2.rviz \
  --ros-args -p use_sim_time:=true
```

Starting RViz **before** the stack is ready can stop the whole thing working
([Problem 6](#problem-6-rviz-breaks-the-whole-stack)). RViz only displays; the
demo works fine without it.

Send a goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Goals worth trying — the obstacles sit at (5, 0), (−3, 4), (0, −5), (3, 3) and
(−4, −3):

| Goal `x, y` | What it shows |
|---|---|
| `2.0, 2.0` | A plain walk across open floor |
| `-2.5, -2.0` | A longer route threading between two obstacles |
| `6.5, 0.0` | Forces a visible detour around the box at (5, 0) |

**What success looks like:** the dog walks a smooth path, RViz shows the planned
route redrawn as obstacles appear, and the command ends with
`Goal finished with status: SUCCEEDED`.

**Expect the dog to stop 0.3–2.1 m from the point you asked for.** That is this
world's localisation accuracy, explained in section 6 — not a bug in the setup.
The larger errors happen on goals that cross unmapped ground, such as
`6.5, 0.0`.

### Mode B — build a new map while driving

```bash
ros2 launch unitree_go2_navigation demo_nav.launch.py mode:=slam
```

Wait **60–90 seconds**. Send goals exactly as above; the map fills in as the dog
explores. When happy, save it from terminal 2:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ws/my_map
```

**What you should see:** `Map saved successfully`, and `my_map.pgm` plus
`my_map.yaml` in `~/go2_jazzy_ws/` on your computer.

Use it later with:

```bash
ros2 launch unitree_go2_navigation demo_nav.launch.py map:=/home/dev/ws/my_map.yaml
```

### Useful options

| Option | Default | Meaning |
|---|---|---|
| `mode:=slam` / `localization` | `localization` | Build a map, or use the saved one |
| `map:=/path/to/map.yaml` | the repo's map | Which map to navigate on |
| `gui:=false` | `true` | Run Gazebo with no window |
| `rviz:=true` | `false` | Start RViz from the launch file (unreliable; see Problem 6) |
| `rviz_delay:=45.0` | `45.0` | Seconds before RViz starts, only when `rviz:=true` |
| `nav2_delay:=40.0` | `40.0` | Seconds before Nav2 starts |
| `odom_source:=ekf` | `ekf` | `lio` is reserved and **not implemented** |

### Shutting down cleanly

Press `Ctrl-C` in terminal 1, then run the cleanup from
[Problem 4](#problem-4-two-simulators-at-once). Leftover processes are the most
common cause of a bad next run.

---

## 6. What is verified, and what is not

### Verified in simulation

- **Both modes work.** `slam_toolbox` owns `map` → `odom` in SLAM mode; AMCL
  owns it in localisation mode; the two EKFs own the rest of the chain. No
  conflicts observed.
- **Exactly one program commands the robot.** `/cmd_vel` consistently shows a
  single publisher, `collision_monitor`.
- **Six out of six navigation goals succeeded**, including two real detours: one
  around the existing box at (5, 0), and one around a 1.2 m block spawned
  directly in the robot's path mid-run.
- **The robot never fell** during any Nav2-driven run.
- **The saved map is accurate.** All five obstacles appear within about 0.5 m of
  their true positions.
- **The lidar keeps up.** Measured over 12 minutes, both headless and with the
  GUI, it produces 10 frames per simulated second as configured, and the gap
  between the clock and the sensor's timestamp stays flat at 0.00–0.12 s.
- **The `odom_source:=lio` stub** prints its warning and falls back to the EKFs
  without disturbing anything.

### NOT verified

- **Nothing has been tested on a real robot.** Every result here is from
  simulation. Do not assume any of it transfers to hardware.
- **Localisation accuracy is poor, and this is the biggest caveat.** Goals
  finished **0.3–2.1 m** from the requested point. The world has only five small
  obstacles, so AMCL has almost nothing to match against; the worst errors came
  when the robot travelled through unmapped space. Nav2 reports `SUCCEEDED`
  because it *believes* it arrived. A world with walls would likely fix this,
  but **that has not been tested.**
- **MPPI was rejected on measurements from this machine only.** It may perform
  fine elsewhere. We did not tune it before switching to DWB.
- **The lidar's 0.8 m blind spot is not solved.** The sensor cannot see anything
  closer than 0.8 m, and the dog is about 0.7 m long, so an obstacle right
  against the robot is invisible. Costmap inflation of 0.70 m partially
  compensates. **Do not expect it to avoid something that walks into it.**
- **The RViz/DDS fix is a workaround.** Not starting RViz from the launch file
  avoids the problem; the underlying discovery saturation was not fixed.
- **Nav2 bringup is not reliable on this machine.** It is intermittent: a
  localization run came up and completed two goals, while a SLAM run with the
  same settings stalled with Nav2 stuck `unconfigured`. The cause is DDS
  dropping lifecycle service replies under startup load
  ([Problem 12](#problem-12-failed-to-bring-up-all-requested-nodes)). **Always
  check `ros2 lifecycle get /bt_navigator` returns `active [3]` before sending
  goals**, and restart if it does not.
- The saved map contains one small false obstacle (31 cells near (2.98, −4.51)).
- **The Velodyne lidar was never used** for navigation. Only the L1 feeds
  `/scan`.

---

## 7. Glossary

**AMCL** — Adaptive Monte Carlo Localisation. Works out where the robot is on a
map you already have, by comparing the live laser scan against it.

**base_footprint** — The robot's position flattened onto the floor, upright
regardless of how the body is tilted. Used as *the* robot position throughout.

**base_link** — The robot's body itself, about 0.22 m above `base_footprint`.

**colcon** — The build tool. `colcon build` copies and compiles everything in
`src/` into `install/`.

**Container** — A sealed box holding an operating system and software, so you do
not install ROS directly on your computer.

**Costmap** — A grid where each square carries a cost: free, blocked, or near
something blocked. The **local** costmap is a small window around the robot used
for immediate steering; the **global** costmap covers the whole map and is used
for route planning.

**DDS** — The messaging system underneath ROS 2, which lets programs find each
other and exchange messages. Saturating it causes Problem 6.

**DWB** — The route-following algorithm used here. Proposes candidate moves,
scores each with "critics", and picks the best.

**EKF** — Extended Kalman Filter. Blends several imperfect measurements (leg
movement, the IMU) into one better position estimate. Two run here.

**Frame** — A named reference point, like `map` or `base_link`. See **TF**.

**Gazebo** — The physics simulator providing the virtual world, robot body and
lidar.

**Inflation** — A safety halo drawn around every obstacle in a costmap so the
planner keeps its distance.

**Launch file** — A Python file that starts several programs at once with the
right settings. All end in `.launch.py`.

**Lidar** — A laser sensor measuring distance in many directions at once. This
robot's is the Unitree L1.

**Lifecycle node** — A program that starts asleep and must be told to configure
and activate. `slam_toolbox` and all Nav2 servers are lifecycle nodes.

**Localisation** — Working out where the robot is on a map it already has.

**Map** — A grid picture of the world: free, blocked or unknown. Saved as a
`.pgm` picture plus a `.yaml` description.

**MPPI** — An alternative route follower, more sophisticated than DWB and more
expensive to run. Rejected here on measurement.

**Nav2** — The ROS 2 navigation system: plans routes, follows them, and recovers
when stuck.

**Node** — One running program in ROS.

**odom** — A frame that drifts slowly over time but never jumps. Good for
short-term motion, not for absolute position.

**Package** — One unit of ROS software, with a `package.xml`. This work added
`unitree_go2_navigation`.

**Parameter** — A setting for a node, usually from a `.yaml` file.

**Point cloud** — A 3D collection of measured points, which is what the lidar
produces before it is flattened.

**ROS 2** — Robot Operating System 2. Not an operating system, but a set of
libraries and conventions letting many small programs cooperate. The version
here is **Jazzy**.

**RViz** — The 3D viewer showing the map, the robot, the laser scan and the
planned route. A display only; it never controls anything.

**SLAM** — Simultaneous Localisation And Mapping. Building a map while working
out where you are in it.

**slam_toolbox** — The SLAM software used here.

**source** — A terminal command that loads settings. Needed in every new
terminal before `ros2` commands work.

**TF** — The system tracking where every frame sits relative to the others. If
TF breaks, everything breaks.

**Topic** — A named channel that programs publish to and subscribe from, like
`/scan` or `/cmd_vel`.

**Workspace** — The folder holding the code: `~/go2_jazzy_ws` on your computer,
`~/ws` inside the container. They are the same folder.

**YAML** — A plain-text settings format, used for all configuration here.

---

## 8. File tree of everything added

```
unitree_go2_ros2/
├── docs/
│   └── NAV2_SETUP_GUIDE.md          This guide
│
├── README.md                        (changed) quick start, TF rules, limitations
│
├── unitree_go2_sim/
│   └── launch/
│       └── unitree_go2_launch.py    (changed) added gui headless wiring and
│                                    the disable_static_map_odom switch
│
└── unitree_go2_navigation/          (new package) all mapping and navigation
    ├── package.xml                  Package ID and its dependency list
    ├── CMakeLists.txt               Installs launch/, config/, maps/, rviz/
    │
    ├── config/
    │   ├── pointcloud_to_laserscan.yaml   Flattens the 3D lidar cloud to a 2D
    │   │                                  /scan; holds the height band
    │   ├── slam_toolbox.yaml              Map-builder settings, laser range
    │   │                                  matched to the L1
    │   └── nav2_params.yaml               All Nav2 settings: AMCL, costmaps,
    │                                      DWB follower, speed limits
    │
    ├── launch/
    │   ├── slam.launch.py           Sim + scan conversion + slam_toolbox.
    │   │                            Mapping only, no Nav2
    │   ├── nav2.launch.py           Adds Nav2. mode:=slam or localization
    │   └── demo_nav.launch.py       One-command demo: the above plus the Nav2
    │                                RViz view, with RViz on a timer
    │
    ├── maps/
    │   ├── default_world.pgm        Saved map picture of the default world
    │   └── default_world.yaml       Its scale, origin and thresholds
    │
    └── rviz/
        └── nav2_go2.rviz            RViz layout for navigation, copied from
                                     nav2_bringup and pointed at base_footprint
```
