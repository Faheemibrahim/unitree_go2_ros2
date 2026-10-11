# Understanding the stack, one stage at a time

> **Status note.** This walkthrough was written against the original `default.sdf`
> world (five small obstacles) and a map of it called `default_world`. That map is
> no longer in the repository: the single maps directory,
> `unitree_go2_navigation/maps/`, now holds `maze_world2`, built by driving the
> robot with teleop in `maze.sdf`, and the navigation launch files default to that
> pair. Where this document says `default_world`, read it as "the map you build
> in these steps". Later changes (the odometry calibration, Regulated Pure
> Pursuit, Smac 2D, the unmapped-obstacle node) are in
> [COMMANDS.md](COMMANDS.md) and the commit messages, not here. The bundled
> `demo_nav.launch.py` and the sim-plus-SLAM `slam.launch.py` it describes no
> longer exist: there are now three launch files (simulator, `slam.launch.py`,
> `nav2.launch.py`), each started separately.

**Read this first.** It explains the robot code as it came, and then each
layer we added on top of it, in the order the layers depend on each other.
For every file it says whether it was **written from scratch**, **copied
from an installed ROS package**, or **an edit to an existing file** — and for
copied files, exactly which lines were changed and why.

The companion [NAV2_SETUP_GUIDE.md](NAV2_SETUP_GUIDE.md) is the reference for
rebuilding everything and troubleshooting. This document is for
*understanding* it. Words like *topic*, *node* and *TF* are defined in that
guide's [glossary](NAV2_SETUP_GUIDE.md#7-glossary).

> **One thing to know up front.** In the git history, stage 2 (getting `/scan`)
> and stage 3 (`slam_toolbox`) went in as a **single commit**, `d8902d2`. They
> are split here because they are two different ideas and you should learn
> them separately. Where one file belongs to both stages, this document says
> which lines belong to which.

---

## Contents

- [The big picture: five stages](#the-big-picture-five-stages)
- [Stage 1 — What came with the robot](#stage-1--what-came-with-the-robot)
- [Stage 2 — Turning the 3D lidar into `/scan`](#stage-2--turning-the-3d-lidar-into-scan)
- [Stage 3 — Building a map with slam_toolbox](#stage-3--building-a-map-with-slam_toolbox)
- [Stage 4 — Where AMCL comes from](#stage-4--where-amcl-comes-from)
- [Stage 5 — Nav2: from a goal to moving legs](#stage-5--nav2-from-a-goal-to-moving-legs)
- [Summary: every file, where it came from](#summary-every-file-where-it-came-from)
- [Check it yourself](#check-it-yourself)

---

## The big picture: five stages

Each stage only works because the one below it works. If something breaks,
start at the bottom.

```
 Stage 5   Nav2              "walk to (2, 2)"  ->  plans a route, steers
             ^  needs: a map, a position, a 2D scan, and a way to move
 Stage 4   AMCL              "where am I on this saved map?"
             ^  needs: a saved map, a 2D scan
 Stage 3   slam_toolbox      "draw me a map while I walk around"
             ^  needs: a 2D scan, odometry
 Stage 2   pointcloud_to_laserscan   3D lidar cloud  ->  flat 2D /scan
             ^  needs: the lidar cloud, the TF tree
 Stage 1   The robot as it came: Gazebo, the body, the sensors,
           CHAMP walking controller, odometry, TF
```

Stages 3 and 4 do the **same job** — answer "where is the robot on the map?"
— in two different situations. You run one or the other, never both.

**What we did *not* touch.** The robot's body model, the walking controller,
the gait settings and the odometry filters are all exactly as they came. All
the new work is one new package, `unitree_go2_navigation`, plus two small edits
to the simulator's launch file.

---

## Stage 1 — What came with the robot

This is the repository at commit `55ff151`, before any of our work.

### The original tree

Only the files that matter for understanding are shown.

```
unitree_go2_ros2/
├── champ/                         CHAMP's core maths library (no ROS nodes)
├── champ_msgs/                    Message types CHAMP uses (Contacts, Pose, ...)
├── champ_base/                    CHAMP's ROS nodes
│   ├── src/quadruped_controller.cpp     turns velocity commands into leg motion
│   ├── src/state_estimation.cpp         works out how far the legs moved the body
│   └── config/
│       ├── ekf/footprint_to_odom.yaml   settings for EKF #1
│       ├── ekf/base_to_footprint.yaml   settings for EKF #2
│       └── velocity_smoother/velocity_smoother.yaml   (UNUSED - see below)
├── unitree_application/
│   └── src/foot_contacts_bridge.cpp     converts Gazebo foot contacts for CHAMP
├── unitree_go2_description/       What the robot IS
│   ├── urdf/unitree_go2_robot.xacro     body, legs, joints
│   ├── urdf/unitree_go2_gazebo.xacro    IMU sensor + ground-truth odometry
│   ├── urdf/lidar_4D_lidar.xacro        the L1 lidar we use
│   ├── urdf/velodyne.xacro              a second lidar (not used by us)
│   ├── urdf/realsense_d455.xacro        a depth camera (not used by us)
│   └── worlds/default.sdf               the world: floor + 5 obstacles
└── unitree_go2_sim/               How to RUN it
    ├── launch/unitree_go2_launch.py     starts everything below
    ├── config/gait/gait.yaml            walking style and SPEED LIMITS
    ├── config/joints/, links/           leg naming for CHAMP
    ├── config/ros_control/              the leg joint controllers
    └── rviz/rviz.rviz                   default viewer layout
```

The easiest way to understand it is to follow what one launch command starts.

### What `unitree_go2_launch.py` starts

`ros2 launch unitree_go2_sim unitree_go2_launch.py` starts these, in roughly
this order:

| # | What starts | What it does |
|---|---|---|
| 1 | **Gazebo** (`gz sim`) with `worlds/default.sdf` | The physics world |
| 2 | **robot_state_publisher** | Reads the robot model, publishes where every body part is relative to `base_link` (the fixed parts of TF) |
| 3 | **create** (spawner) | Drops the robot into the world at height 0.375 m |
| 4 | **ros_gz_bridge** × 5 | Copies Gazebo's internal data into ROS topics (see below) |
| 5 | **quadruped_controller_node** (CHAMP) | Listens for velocity commands, moves the legs |
| 6 | **foot_contacts_bridge** | Tells CHAMP which feet are on the ground |
| 7 | **state_estimation_node** (CHAMP) | Estimates body motion from the legs |
| 8 | **EKF #1** `footprint_to_odom_ekf` | Publishes `odom` → `base_footprint` |
| 9 | **EKF #2** `base_to_footprint_ekf` | Publishes `base_footprint` → `base_link` |
| 10 | **static_transform_publisher** `map_to_odom_tf_node` | Publishes a fake, fixed `map` → `odom` |
| 11 | **RViz** | The viewer |
| 12 | Leg controller spawners, at **t = 20 s** and **t = 30 s** | Switch the leg joints on |

> **Why the long start-up wait.** Line 12: the controller that actually moves
> the legs is only switched on **30 seconds** after launch. Before that the
> robot cannot walk, whatever you tell it. Everything we add later has to wait
> for this.

### How sensor data gets out of Gazebo

Gazebo runs as its own program with its own internal messaging. The
**ros_gz_bridge** nodes copy data across into ROS. The original launch file
bridges:

| ROS topic | Comes from | Rate (measured) | Frame |
|---|---|---|---|
| `/clock` | Gazebo's simulated time | — | — |
| `/imu/data` | IMU in `unitree_go2_gazebo.xacro` | ~92 Hz | `imu_link` |
| `/unitree_lidar/points` | L1 lidar in `lidar_4D_lidar.xacro` | ~8.9 Hz | `lidar_l1_link` |
| `/odom/ground_truth` | Gazebo's perfect position (cheating, for testing only) | 50 Hz | `odom` |
| `/lf_foot_contacts` etc. | Foot contact sensors | — | — |
| `/velodyne_points/points`, `/d455/...`, `/rgb_image` | Other sensors we do not use | — | — |

**The lidar is the thing everything later depends on.** From
`unitree_go2_description/urdf/lidar_4D_lidar.xacro`:

- 600 samples around a full circle × 30 rows covering ±45° up and down
  = **18,000 points** per sweep
- sees from **0.8 m** to **30 m** — it is blind closer than 0.8 m
- 10 sweeps per simulated second
- mounted 0.25 m forward on the body, **tilted at roll ≈ 165°, yaw ≈ 90°**
  (nearly upside down)

That tilt is the reason stage 2 has to do what it does.

### How the robot walks: the command path

```
  /cmd_vel   (a Twist: forward speed, sideways speed, turn rate)
      |
      v
  quadruped_controller_node            (CHAMP)
      |   uses gait.yaml: max 0.3 m/s forward, 0.25 m/s sideways, 0.5 rad/s turn
      v
  /joint_group_effort_controller/joint_trajectory
      |
      v
  gz_ros2_control  ->  leg motors in Gazebo
```

Two oddities here, both inherited:

1. **The controller's input is really called `cmd_vel/smooth`, not
   `cmd_vel`.** The launch file renames it:
   `remappings=[("/cmd_vel/smooth", "/cmd_vel")]`. The name is a leftover
   from older CHAMP, where a smoother sat in front of the controller.
2. **That smoother no longer exists in the chain.**
   `champ_base/config/velocity_smoother/velocity_smoother.yaml` is still in the
   repository, but **no launch file starts anything that uses it**. So in the
   original code, whatever you publish on `/cmd_vel` goes straight to the legs,
   unsmoothed. This is why sudden commands make the robot fall over, and why
   stage 5 adds its own smoother.

### How the robot knows where it is: odometry and TF

There is no GPS-style "true position" in normal use. The robot works it out
from its legs and its IMU:

```
  joint angles + foot contacts
      |
      v
  state_estimation_node (CHAMP)
      |                                  \
      | /odom/raw                         | /base_to_footprint_pose
      | (how fast the body is moving)     | (how the body sits over its feet)
      v                                   v
  EKF #1  footprint_to_odom_ekf          EKF #2  base_to_footprint_ekf
      + /imu/data (heading, turn rate)       + /imu/data (tilt, tilt rate)
      |                                   |
      | TF:  odom -> base_footprint       | TF:  base_footprint -> base_link
      | topic: /odom                      | topic: /odom/local
```

An **EKF** blends several imperfect measurements into one better estimate.
What each one reads, from `champ_base/config/ekf/`:

- **EKF #1** (`footprint_to_odom.yaml`) takes forward/sideways speed and turn
  rate from the legs (`odom/raw`), and heading plus turn rate from the IMU. It
  runs in **2D mode** — it only cares about position on the floor and heading.
  This is the robot's **odometry**: good over short distances, drifts slowly
  over long ones.
- **EKF #2** (`base_to_footprint.yaml`) takes the body's height and tilt from
  CHAMP, and tilt from the IMU. It describes how the body is lifted and leaning
  above the floor point.

The resulting TF tree, as it came:

```
  map                 <- fake: static_transform_publisher says "map = odom, always"
   |
  odom                <- EKF #1
   |
  base_footprint      <- EKF #2       (the robot's spot on the floor, kept level)
   |
  base_link           <- ~0.219 m up  (the body)
   |
  lidar_l1_link, imu_link, legs ...   <- robot_state_publisher (fixed by the model)
```

**The fake `map` → `odom` matters.** It claims the odometry never drifts. That
is fine for driving around with a keyboard, but it is a lie, and stage 3
exists to replace it with the truth.

### What the robot can already do at the end of stage 1

- Walk when you publish on `/cmd_vel` (e.g. with `teleop_twist_keyboard`)
- Report a 3D lidar cloud, IMU, odometry and a full TF tree

What it **cannot** do: draw a map, know where it is on a map, or go anywhere
by itself.

---

## Stage 2 — Turning the 3D lidar into `/scan`

### Why this stage exists

`slam_toolbox`, AMCL and Nav2's costmaps all want a **2D laser scan**: one flat
ring of distances, like a single slice through the room at waist height. The
L1 gives us 18,000 points in 3D. So we need a converter that cuts a horizontal
slice out of the cloud and flattens it.

That converter already exists as an installed ROS package,
**`pointcloud_to_laserscan`**. We did not write it. We wrote its settings, and
a launch file to start it.

```
  /unitree_lidar/points    3D: 18,000 points, frame lidar_l1_link (tilted)
         |
         v
  pointcloud_to_laserscan
     1. convert every point into base_footprint (level, on the floor)
     2. keep only points between 0.25 m and 1.1 m above the floor
     3. for each of 600 directions, keep the nearest point
         |
         v
  /scan                    2D: 601 distances in a circle, frame base_footprint
```

### Files added in this stage

```
unitree_go2_navigation/                        NEW PACKAGE
├── package.xml                                written from scratch
├── CMakeLists.txt                             written from scratch
├── config/
│   └── pointcloud_to_laserscan.yaml           written from scratch
└── launch/
    └── slam.launch.py                         written; only PART of it is stage 2
```

#### `package.xml` and `CMakeLists.txt` — written from scratch

Every ROS package needs these two. They were modelled on the existing
`unitree_go2_sim` package, which is also "configuration and launch files only,
no code", so they follow the same pattern.

- `package.xml` names the package and lists what it needs to run. The line
  relevant to this stage is `<exec_depend>pointcloud_to_laserscan</exec_depend>`.
- `CMakeLists.txt` compiles nothing. It only copies the `launch/` and
  `config/` folders to where ROS looks for them at run time. **If a folder is
  not listed here, ROS cannot find it**, even though it is right there on disk.

#### `config/pointcloud_to_laserscan.yaml` — written from scratch

**Where it came from:** the `pointcloud_to_laserscan` package ships **no
config file**. It only ships an example launch file,
`/opt/ros/jazzy/share/pointcloud_to_laserscan/launch/sample_pointcloud_to_laserscan_launch.py`,
with the settings written inline. We used the same parameter names and chose
every value for this robot:

| Parameter | Example's value | Our value | Why ours |
|---|---|---|---|
| `target_frame` | `cloud` | **`base_footprint`** | The lidar is mounted nearly upside down. "Height" only means "height above the floor" after converting to a level frame on the floor. |
| `min_height` | `0.0` | **`0.25`** | The floor shows up at about −0.05 m in `base_footprint`. A walking dog's body rocks, which tilts far-away floor points upward. 0.25 m keeps the floor out. |
| `max_height` | `1.0` | **`1.1`** | Obstacles in this world are 1 m tall; this keeps their full height. |
| `angle_min` / `angle_max` | ±1.5708 (half circle) | **±3.14159** (full circle) | The L1 sees all the way round |
| `angle_increment` | 0.0087 | **0.01047** | 2π ÷ 600: one slot per lidar sample |
| `scan_time` | 0.3333 | **0.1** | The lidar sweeps 10 times per second |
| `range_min` | 0.45 | **0.8** | The L1 is blind closer than 0.8 m |
| `range_max` | 4.0 | **30.0** | The L1's real maximum |
| `transform_tolerance` | 0.01 | **0.1** | More slack for the simulator's timing |
| `use_inf`, `inf_epsilon` | `True`, `1.0` | same | "Nothing seen" is reported as infinity, not as a fake wall |
| `use_sim_time` | (not set) | **`true`** | Use Gazebo's clock. Almost every timing bug in simulation is this missing. |
| `queue_size` | (not set) | `10` | Small buffer of incoming clouds |

> The height band was first `0.1`–`0.8` (commit `d8902d2`) and raised to
> `0.25`–`1.1` in commit `6cee848` after we measured where the floor actually
> appeared. With the robot standing upright, the current band keeps **383 of
> the 18,000 points**, all at 3.6–5.3 m — exactly where the real obstacles are.

#### `launch/slam.launch.py` — the stage-2 part

Despite its name, the first half of this file is pure stage 2. These are the
two blocks that produce `/scan`:

```python
# 1. Start the robot as it came in stage 1, by running its own launch file.
sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(unitree_go2_sim, "launch", "unitree_go2_launch.py")
    ),
    launch_arguments={
        "gui": LaunchConfiguration("gui"),
        "rviz": LaunchConfiguration("rviz"),
        "disable_static_map_odom": "true",   # <- this line belongs to stage 3
    }.items(),
)

# 2. Start the converter.
pointcloud_to_laserscan_node = Node(
    package="pointcloud_to_laserscan",
    executable="pointcloud_to_laserscan_node",
    name="pointcloud_to_laserscan",
    output="screen",
    parameters=[pointcloud_to_laserscan_config],      # the yaml above
    remappings=[
        ("cloud_in", "/unitree_lidar/points"),
        ("scan", "/scan"),
    ],
)
```

- **`IncludeLaunchDescription`** — instead of copying all of stage 1 into our
  file, we just run the original launch file from inside ours. So stage 1 is
  reused untouched.
- **`remappings`** — the converter has fixed names for its input (`cloud_in`)
  and output (`scan`). Remapping connects them to this robot's real topics
  **without editing the converter's code**. This is the normal way to wire ROS
  programs together.
- **`name="pointcloud_to_laserscan"`** must match the first line of the yaml
  file, `pointcloud_to_laserscan:`. That is how a node finds its own settings
  inside a yaml file.

### Changes to existing files in this stage

**None are needed to get `/scan`.** Stage 1's lidar bridge already publishes
`/unitree_lidar/points`, and the TF tree already connects `lidar_l1_link` to
`base_footprint`.

There was one **unrelated convenience edit** at the same time, commit
`bc92a67`, in `unitree_go2_sim/launch/unitree_go2_launch.py`: the simulator
had a `gui` on/off option that was never wired to anything. We wired it, so
`gui:=false` runs Gazebo with no window (`-s --headless-rendering`). It is not
required for anything; it only made automated testing possible.

### What you can see at the end of stage 2

```bash
ros2 topic list | grep -E "^/scan$|unitree_lidar"
ros2 topic echo /scan --qos-reliability best_effort --once
```

You should see `/scan` with `frame_id: base_footprint`. (`--qos-reliability
best_effort` is needed because sensor topics drop messages rather than wait;
without it the command prints nothing even though `/scan` is working.)

---

## Stage 3 — Building a map with slam_toolbox

### Why this stage exists

Now we have a flat scan. **SLAM** (Simultaneous Localisation And Mapping)
uses it to draw a floor plan *and* keep track of where the robot is on that
floor plan, at the same time.

It also fixes stage 1's lie. Remember the fake `map` → `odom` that claims
odometry never drifts? `slam_toolbox` replaces it with a real one: it compares
each new scan with the map it has drawn so far, notices how far odometry has
drifted, and publishes the correction as `map` → `odom`.

```
  /scan  +  TF (odom -> base_footprint, from EKF #1)
         |
         v
     slam_toolbox
         |                         \
         | /map                     | TF:  map -> odom   (the drift correction)
         | the floor plan           |      REPLACES stage 1's fake one
         v
   map_saver_cli  ->  default_world.pgm + default_world.yaml
```

**The one rule:** only one program may publish `map` → `odom`. If both
`slam_toolbox` and stage 1's fake publisher do, the robot's position jumps
between two answers and nothing works. So this stage must switch the fake one
off.

### Files changed in this stage

#### `unitree_go2_sim/launch/unitree_go2_launch.py` — edit to an existing file

Commit `dc66f50`. Adds an off-switch for stage 1's fake `map` → `odom`:

```diff
+    declare_disable_static_map_odom = DeclareLaunchArgument(
+        "disable_static_map_odom",
+        default_value="false",
+        description="Leave out the static map->odom transform, for when SLAM publishes it",
+    )
     ...
     map_to_odom_tf_node = Node(
         package='tf2_ros',
         ...
             '--frame-id', 'map', '--child-frame-id', 'odom'
         ],
+        condition=UnlessCondition(LaunchConfiguration("disable_static_map_odom")),
     )
```

- `default_value="false"` means **running the simulator on its own behaves
  exactly as before**. The fake link is only removed when something asks.
- `UnlessCondition` means "start this node *unless* the switch is on".
- `slam.launch.py` turns the switch on with `"disable_static_map_odom": "true"`
  (the line marked "belongs to stage 3" above).

### Files added in this stage

```
unitree_go2_navigation/
├── config/
│   └── slam_toolbox.yaml          COPIED from slam_toolbox, 3 lines changed
├── launch/
│   └── slam.launch.py             second half: slam_toolbox (pattern copied)
└── maps/                          (commit b7d44d6)
    ├── default_world.pgm          produced by map_saver_cli
    └── default_world.yaml         produced by map_saver_cli
```

#### `config/slam_toolbox.yaml` — copied, then 3 lines changed

**Copied from:**
`/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml`

Comparing the two files (ignoring comments and blank lines) gives **exactly
three differences**:

```diff
 slam_toolbox:
   ros__parameters:
+    use_sim_time: true
 ...
-    min_laser_range: 0.0
-    max_laser_range: 20.0
+    min_laser_range: 0.8
+    max_laser_range: 30.0
```

| Change | Why |
|---|---|
| `use_sim_time: true` added | Use Gazebo's clock, not your computer's |
| `min_laser_range` 0.0 → **0.8** | The L1 is blind under 0.8 m; the map must not invent readings there |
| `max_laser_range` 20.0 → **30.0** | The L1 really does see 30 m |

**Everything else is the stock file, unchanged on purpose.** It already
expects `base_frame: base_footprint`, `odom_frame: odom`, `map_frame: map`
and `scan_topic: /scan` — which happen to be exactly this robot's names. Two
settings worth knowing about, both stock:

- `resolution: 0.05` — each map square is 5 cm.
- `minimum_travel_distance: 0.5` — only add a scan to the map after moving
  0.5 m, so standing still does not fill the map with duplicates.

#### `launch/slam.launch.py` — the stage-3 part

**Where the pattern came from:** `slam_toolbox`'s own launch file,
`/opt/ros/jazzy/share/slam_toolbox/launch/online_async_launch.py`. We copied
its approach to starting the node, but not the file itself.

```python
slam_toolbox_node = LifecycleNode(                  # not a plain Node
    package="slam_toolbox",
    executable="async_slam_toolbox_node",
    name="slam_toolbox",                             # matches "slam_toolbox:" in the yaml
    namespace="",
    output="screen",
    parameters=[slam_toolbox_config, {"use_lifecycle_manager": False}],
)

configure_slam_toolbox = EmitEvent(                 # wake-up call #1
    event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_toolbox_node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    )
)

activate_slam_toolbox = RegisterEventHandler(       # wake-up call #2, after #1 finishes
    OnStateTransition(
        target_lifecycle_node=slam_toolbox_node,
        start_state="configuring",
        goal_state="inactive",
        entities=[ ... TRANSITION_ACTIVATE ... ],
    )
)
```

**Why it looks so complicated.** On ROS 2 Jazzy, `slam_toolbox` is a
*lifecycle node*: it starts **asleep** and does nothing until told
"configure" and then "activate". Our first version used a plain `Node` and
it sat there forever producing no map, with no error message. These two
blocks are the wake-up calls, copied from the official launch file.

What we left out of the official file: its `autostart` and
`use_lifecycle_manager` launch options. We always want it to start, so we
fixed those values instead of making them options.

#### `maps/default_world.pgm` and `.yaml` — produced, not written

Generated by driving the robot around with `slam.launch.py` running and then
saving:

```bash
ros2 run nav2_map_server map_saver_cli -f default_world
```

The `.pgm` is a greyscale picture (white = free, black = blocked, grey =
unknown). The `.yaml` says how big each pixel is (5 cm) and where the
picture's corner sits in the world. Checked against the world file: all five
obstacles appear within about 0.5 m of their true positions.

### What you can see at the end of stage 3

```bash
ros2 lifecycle get /slam_toolbox                  # expect: active [3]
ros2 topic info /tf --verbose | grep "Node name"  # slam_toolbox is now a publisher
ros2 node list | grep map_to_odom                 # expect: nothing (fake one is off)
```

---

## Stage 4 — Where AMCL comes from

### The short answer

**We wrote no AMCL code and no AMCL launch file.** AMCL is a program inside
Nav2. It arrives in three pieces, none of them ours except some settings:

| Piece | Where it lives | Ours? |
|---|---|---|
| The AMCL program | package `nav2_amcl`, installed by `ros-jazzy-navigation2` in the Dockerfile | No |
| The launch file that starts it | `/opt/ros/jazzy/share/nav2_bringup/launch/localization_launch.py` | No, used unmodified |
| Its settings | the `amcl:` section of **our** `config/nav2_params.yaml` | Copied from Nav2, one addition |

### Why AMCL exists when we already have slam_toolbox

They answer the same question in two different situations:

| | slam_toolbox (stage 3) | AMCL (stage 4) |
|---|---|---|
| Do you have a map already? | No — it draws one | **Yes** — it loads a saved one |
| What it publishes | `/map` and `map` → `odom` | `map` → `odom` only |
| When to use | Exploring a new place | Working in a place you mapped before |

Both publish `map` → `odom`. **Run one or the other, never both.**

### How it gets started

`nav2.launch.py` (stage 5) contains this, which runs Nav2's stock
localisation launch file:

```python
localization = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(nav2_bringup, "launch", "localization_launch.py")
    ),
    launch_arguments={
        "map": LaunchConfiguration("map"),     # our maps/default_world.yaml
        "use_sim_time": "true",
        "params_file": nav2_params,            # our config/nav2_params.yaml
    }.items(),
    condition=UnlessCondition(is_slam),        # ONLY when mode is not "slam"
)
```

That stock file starts **three** programs (its `lifecycle_nodes` list is
`['map_server', 'amcl']`, plus the manager itself):

```
  localization_launch.py   (Nav2's, unmodified)
     |
     +-- map_server              loads default_world.yaml, publishes it as /map
     +-- amcl                    reads /map + /scan + odometry,
     |                           publishes map -> odom
     +-- lifecycle_manager_localization
                                 wakes the two above (same "asleep at start"
                                 idea as slam_toolbox)
```

And `condition=UnlessCondition(is_slam)` is what guarantees AMCL and
`slam_toolbox` are never both running: `slam_toolbox` has the opposite
condition, `IfCondition(is_slam)`.

### How AMCL works, in one paragraph

It scatters 500–2,000 guesses ("particles") of where the robot might be on
the map. Each time the robot moves, it moves every guess the same way using
odometry. Each time a scan arrives, it scores every guess by how well the scan
would line up with the map from that spot, and throws away the bad ones. The
surviving guesses cluster around the true position. The difference between
that and where odometry thinks the robot is becomes `map` → `odom`.

### The AMCL settings: what we changed

**Copied from:** the `amcl:` section of
`/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`.

The only change is an addition:

```diff
     scan_topic: scan
+
+    # The robot always spawns at the map origin in this world, so seed AMCL
+    # there instead of requiring a manual "2D Pose Estimate" in RViz.
+    set_initial_pose: true
+    initial_pose:
+      x: 0.0
+      y: 0.0
+      z: 0.0
+      yaw: 0.0
```

**Why:** AMCL needs a starting guess. Without one you must click
"2D Pose Estimate" in RViz on every run. The robot always spawns at the map's
origin, so we just tell it so.

**What we did not need to change, and why:** the stock settings already say
`base_frame_id: "base_footprint"`, `odom_frame_id: "odom"`,
`global_frame_id: "map"` and `scan_topic: scan`. Those happen to be this
robot's names exactly.

**What is left at stock and not tuned:** everything else, including
`laser_max_range: 100.0`, `max_beams: 60` and the motion-noise values
`alpha1`–`alpha5`. Unlike `slam_toolbox`, AMCL's laser range was **not**
matched to the L1. This is one of the possible reasons AMCL's accuracy is
poor here (goals end 0.3–2.1 m off); the main reason is that the world has
only five small obstacles to match against. **Neither has been tested.**

---

## Stage 5 — Nav2: from a goal to moving legs

### Why this stage exists

Stages 3 and 4 tell the robot *where it is*. Nav2 decides *where to go and
how to get there*, and sends the walking commands.

### How a goal travels through the system

This is the workflow every time you send a goal. Follow the numbers.

```
 (1) You:  ros2 action send_goal /navigate_to_pose ... x: 2.0, y: 2.0
            |
            v
 (2) bt_navigator          the "manager": runs a behaviour tree that says
            |              plan -> follow -> if stuck, recover -> retry
            |
            |-- asks -->  (3) planner_server  (NavFn)
            |                  uses the GLOBAL costmap:
            |                    static_layer    = /map  (from map_server or slam_toolbox)
            |                    obstacle_layer  = /scan (things not on the map)
            |                    inflation_layer = 0.70 m keep-out halo
            |                  returns a route: a list of points from here to the goal
            |
            |-- asks -->  (4) controller_server  (DWB)
                               uses the LOCAL costmap: 6 m x 6 m around the robot,
                               from /scan only
                               20 times a second, tries 20 x 20 candidate
                               speed/turn combinations, imagines each 2 s ahead,
                               scores them, picks the best
                               |
                               | /cmd_vel_nav
                               v
                          (5) velocity_smoother
                               caps speed and acceleration so the dog does not trip
                               |
                               | /cmd_vel_smoothed
                               v
                          (6) collision_monitor
                               last safety check; the ONLY publisher of /cmd_vel
                               |
                               | /cmd_vel
 ================================|========================  stage 1 from here
                               v
                          (7) quadruped_controller_node (CHAMP) -> legs move
                               |
                               v
                          (8) legs move -> state_estimation -> EKF #1
                               -> odom -> base_footprint updates
                               |
                               v
                          (9) AMCL / slam_toolbox see the new scan
                               -> map -> odom correction updates
                               |
                               v
                          back to (4): the controller sees the robot's new
                          position and picks the next command
```

When the robot is close enough to the goal (within 0.25 m, by its own
estimate), `bt_navigator` reports `SUCCEEDED`.

**If it gets stuck** — the controller cannot make 0.5 m of progress in 10
seconds — `bt_navigator` triggers recovery behaviours from
`behavior_server`: spin on the spot, back up, wait, then re-plan.

### Files added in this stage

```
unitree_go2_navigation/
├── config/
│   └── nav2_params.yaml        COPIED from nav2_bringup, then edited (below)
├── launch/
│   ├── nav2.launch.py          written; pattern from nav2_bringup
│   └── demo_nav.launch.py      written from scratch
└── rviz/
    └── nav2_go2.rviz           COPIED from nav2_bringup, UNCHANGED
```

#### `config/nav2_params.yaml` — copied, then edited

**Copied from:** `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`.

That stock file has settings for every Nav2 program, written for a small
wheeled robot (a TurtleBot, radius 0.22 m, top speed 0.5 m/s). A full
`diff` against it touches eight places. Here is each one, in file order, with
what changed and why.

**1. AMCL — starting position added.** Covered in stage 4.

**2. `bt_navigator` — which frame is "the robot"**

```diff
-    robot_base_frame: base_link
+    robot_base_frame: base_footprint
```

`base_link` is the body, 0.22 m up and tilting as it walks. `base_footprint`
is the level point on the floor. `slam_toolbox` and AMCL both use
`base_footprint`, so Nav2 must too, or the robot's position would disagree
between programs. **The same one-line change is made in four places:**
`bt_navigator`, both costmaps, and `collision_monitor`.

**3. `controller_server` — the route follower: MPPI replaced by DWB**

This is the biggest change. The stock file uses a follower called **MPPI**.
The whole MPPI block was deleted and replaced with a **DWB** block:

```yaml
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.26          # forward, just under CHAMP's 0.3 limit
      max_vel_y: 0.0           # no sideways stepping
      max_vel_theta: 0.45      # turning, just under CHAMP's 0.5 limit
      acc_lim_x: 0.5           # gentle speed-ups
      acc_lim_theta: 1.0
      decel_lim_x: -0.7
      decel_lim_theta: -1.4
      vx_samples: 20           # try 20 forward speeds ...
      vtheta_samples: 20       # ... times 20 turn rates
      sim_time: 2.0            # imagine each one 2 s into the future
      xy_goal_tolerance: 0.25  # "arrived" means within 0.25 m
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign",
                "PathAlign", "PathDist", "GoalDist"]
```

**Why not keep MPPI:** we measured it. On this machine, alongside Gazebo, it
managed only about **5.5 updates per second** instead of 20, and commanded
only about **0.06 m/s**. Nav2 then decided the robot was stuck and started
spinning it around. DWB is much cheaper and worked. This is a judgement about
this computer under this load; MPPI may be better elsewhere.

**Why these speeds:** CHAMP's own limits in
`unitree_go2_sim/config/gait/gait.yaml` are 0.3 m/s forward, 0.25 m/s
sideways and 0.5 rad/s turning. Going over them makes the dog fall.

**Why `max_vel_y: 0.0`:** the dog *can* step sideways, but forbidding it keeps
the gait more stable and makes DWB's job simpler.

**What the critics are:** each candidate move gets a score from each critic,
and the best total wins. They reward staying on the route (`PathDist`,
`PathAlign`), heading towards the goal (`GoalDist`, `GoalAlign`), and turning to
the right final direction (`RotateToGoal`); they punish getting near obstacles
(`BaseObstacle`) and dithering back and forth (`Oscillation`).

**4. `local_costmap` — what is around the robot right now**

```diff
       global_frame: odom
-      robot_base_frame: base_link
+      robot_base_frame: base_footprint
       rolling_window: true
-      width: 3
-      height: 3
+      width: 6
+      height: 6
       resolution: 0.05
-      robot_radius: 0.22
-      plugins: ["voxel_layer", "inflation_layer"]
+      robot_radius: 0.35
+      plugins: ["obstacle_layer", "inflation_layer"]
       ...
-          raytrace_max_range: 3.0
+          raytrace_max_range: 5.0
-          obstacle_max_range: 2.5
+          obstacle_max_range: 4.0
```

| Change | Why |
|---|---|
| `width`/`height` 3 → **6** m | The L1 sees much further than the TurtleBot's lidar; a bigger window uses more of it |
| `robot_radius` 0.22 → **0.35** m | The dog standing is about 0.65 × 0.35 m; 0.35 m is the circle that contains it |
| `voxel_layer` → **`obstacle_layer`** | A voxel layer is a 3D grid. Our `/scan` is flat 2D, so 3D is wasted work |
| ranges 3.0 / 2.5 → **5.0 / 4.0** m | Mark and clear obstacles further out, inside the bigger window |
| `inflation_radius: 0.70` | **Unchanged.** Already bigger than the new radius, so the planner keeps a safe gap |

`rolling_window: true` means the 6 × 6 m square moves with the robot. It has
no saved map in it — only what the lidar sees now.

**5. `global_costmap` — the whole known world**

```diff
-      robot_base_frame: base_link
-      robot_radius: 0.22
+      robot_base_frame: base_footprint
+      robot_radius: 0.35
       ...
-          raytrace_max_range: 3.0
+          raytrace_max_range: 10.0
-          obstacle_max_range: 2.5
+          obstacle_max_range: 8.0
```

Same frame and radius reasons. Its three layers, unchanged from stock, are
`static_layer` (the saved or SLAM map), `obstacle_layer` (new things in
`/scan`) and `inflation_layer` (the keep-out halo).

**6. `planner_server` — unchanged.** It uses `NavfnPlanner` (shortest path
over the costmap) with `allow_unknown: true`, so it may plan through areas
the map has not seen. A dog can turn on the spot, so it does not need the
car-style planners.

**7. `collision_monitor` — frame only.** `base_link` → `base_footprint`.
Its topics were already right in the stock file:
`cmd_vel_in_topic: "cmd_vel_smoothed"`, `cmd_vel_out_topic: "cmd_vel"`. That
is what makes it the last link in the chain.

**8. `velocity_smoother` — the dog's real limits**

```diff
-    max_velocity: [0.5, 0.0, 2.0]
-    min_velocity: [-0.5, 0.0, -2.0]
-    max_accel: [2.5, 0.0, 3.2]
-    max_decel: [-2.5, 0.0, -3.2]
+    max_velocity: [0.30, 0.25, 0.50]
+    min_velocity: [-0.20, -0.25, -0.50]
+    max_accel: [0.40, 0.40, 0.80]
+    max_decel: [-0.60, -0.60, -1.20]
```

Each list is `[forward, sideways, turn]`. Speeds come from `gait.yaml`.
Accelerations are cut by a factor of about six from stock: the stock values
would jerk the dog off its feet. **This is the smoother stage 1 was missing.**

> A small inconsistency, harmless: the smoother allows 0.25 m/s sideways, but
> DWB never asks for any sideways speed (`max_vel_y: 0.0`), so it never
> happens.

#### `launch/nav2.launch.py` — written; pattern from nav2_bringup

This file combines everything. In order, it starts:

1. **Stage 1** — the simulator, with the fake `map` → `odom` switched off
2. **Stage 2** — `pointcloud_to_laserscan`
3. **Stage 3 *or* 4** — `slam_toolbox` if `mode:=slam`, otherwise Nav2's
   stock `localization_launch.py` (map_server + AMCL)
4. **After a 40-second wait** — the Nav2 programs

**Why the wait:** the legs are only switched on at 30 seconds (stage 1). Start
Nav2 sooner and its costmaps look for a TF tree that does not exist yet, and
its start-up requests time out under the load.

**Why Nav2 is started program by program.** Nav2 ships a launch file that
starts all its programs at once,
`/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py`. Its list is:

```
controller_server, smoother_server, planner_server, route_server,
behavior_server, velocity_smoother, collision_monitor, bt_navigator,
waypoint_follower, docking_server
```

We tried it first. **`docking_server` also publishes `/cmd_vel`**, which gave
the robot two programs issuing commands. So `nav2.launch.py` starts these
seven by hand instead, leaving out `docking_server`, `route_server` and
`waypoint_follower` (none of which this demo needs):

```
controller_server, smoother_server, planner_server, behavior_server,
bt_navigator, velocity_smoother, collision_monitor
```

Each one gets the same settings file and the same `/cmd_vel` re-routing as
Nav2's own launch file uses:

```python
to_cmd_vel_nav = [("cmd_vel", "cmd_vel_nav")]
```

That remapping is what forces the controller's output through the smoother
and the collision monitor, instead of straight to the legs.

#### `launch/demo_nav.launch.py` — written from scratch

A convenience wrapper. It runs `nav2.launch.py` with demo-friendly defaults
(`mode:=localization`, so it uses the saved map) and adds an `odom_source`
option that is a **placeholder** for later laser-based odometry — setting it
to `lio` only prints a warning.

It *can* start RViz, but **does not by default** (`rviz:=false`): starting
RViz from the launch file overloaded the system's message discovery and
stopped Nav2 from starting. Start RViz by hand once everything is running.

#### `rviz/nav2_go2.rviz` — copied, unchanged

**Copied from:** `/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz`.

**It is byte-for-byte identical to the original.** It is a viewer layout only:
which panels and displays RViz opens with. Its fixed frame is `map`, which is
already right for this robot, so nothing needed changing.

---

## Summary: every file, where it came from

### Edits to files that came with the robot

| File | Commit | Change | Stage |
|---|---|---|---|
| `unitree_go2_sim/launch/unitree_go2_launch.py` | `bc92a67` | wired the unused `gui` switch to headless mode | convenience |
| `unitree_go2_sim/launch/unitree_go2_launch.py` | `dc66f50` | added `disable_static_map_odom` | 3 |

**Nothing else that came with the robot was changed** — not the robot model,
not CHAMP, not the gait, not the EKFs.

### New files

| File | Origin | Changed from origin | Stage |
|---|---|---|---|
| `unitree_go2_navigation/package.xml` | written | — | 2 |
| `unitree_go2_navigation/CMakeLists.txt` | written | — | 2 |
| `config/pointcloud_to_laserscan.yaml` | written; parameter names from the package's sample launch file | every value | 2 |
| `launch/slam.launch.py` | written; slam_toolbox part follows `slam_toolbox/launch/online_async_launch.py` | — | 2 + 3 |
| `config/slam_toolbox.yaml` | **copied**: `slam_toolbox/config/mapper_params_online_async.yaml` | 3 lines | 3 |
| `maps/default_world.pgm`, `.yaml` | **generated** by `map_saver_cli` | — | 3 |
| *(AMCL itself)* | **installed**: `nav2_amcl`, started by `nav2_bringup/launch/localization_launch.py` | not ours | 4 |
| `config/nav2_params.yaml` | **copied**: `nav2_bringup/params/nav2_params.yaml` | 8 places, incl. MPPI → DWB | 4 + 5 |
| `launch/nav2.launch.py` | written; follows `nav2_bringup/launch/navigation_launch.py` | — | 4 + 5 |
| `launch/demo_nav.launch.py` | written | — | 5 |
| `rviz/nav2_go2.rviz` | **copied**: `nav2_bringup/rviz/nav2_default_view.rviz` | **none** | 5 |

All "copied" paths are under `/opt/ros/jazzy/share/` inside the container.

---

## Check it yourself

You do not have to trust this document. Every claim about a copied file can be
checked by comparing it against the original. Run these inside the container:

```bash
source /opt/ros/jazzy/setup.bash
R=~/ws/src/unitree_go2_ros2/unitree_go2_navigation
S=/opt/ros/jazzy/share
```

**Stage 2** — the example settings our yaml was based on:

```bash
cat $S/pointcloud_to_laserscan/launch/sample_pointcloud_to_laserscan_launch.py
```

**Stage 3** — expect exactly the three differences shown above. The `clean`
helper strips comments and trailing spaces first; the stock file has many
end-of-line comments that would otherwise drown out the real changes:

```bash
clean() { grep -vE '^\s*#|^\s*$' "$1" | sed 's/\s*#.*//; s/[[:space:]]*$//'; }
diff <(clean $S/slam_toolbox/config/mapper_params_online_async.yaml) \
     <(clean $R/config/slam_toolbox.yaml)
```

**Stage 4** — the stock launch file that starts AMCL:

```bash
grep -n "lifecycle_nodes =" $S/nav2_bringup/launch/localization_launch.py
```

**Stage 5** — every change to the Nav2 settings, and proof the RViz file is
unchanged:

```bash
diff -u $S/nav2_bringup/params/nav2_params.yaml $R/config/nav2_params.yaml | less
diff $S/nav2_bringup/rviz/nav2_default_view.rviz $R/rviz/nav2_go2.rviz && echo IDENTICAL
```

**What changed in the robot's own files**, from the start of our work:

```bash
cd ~/ws/src/unitree_go2_ros2
git diff 55ff151 -- unitree_go2_sim unitree_go2_description champ_base champ
```

That last command should show changes **only** in
`unitree_go2_sim/launch/unitree_go2_launch.py`.

**Seeing each stage running** — after
`ros2 launch unitree_go2_navigation demo_nav.launch.py` and a 90–120 s wait:

```bash
ros2 topic list | grep -E "unitree_lidar|^/scan$|^/map$|cmd_vel"   # stages 1, 2, 3/4, 5
ros2 node list | grep -E "pointcloud|amcl|map_server|slam|bt_nav|controller_server"
ros2 topic info /cmd_vel --verbose | grep "Node name"               # collision_monitor + CHAMP
ros2 run tf2_tools view_frames                                      # writes frames.pdf
```
