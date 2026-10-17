"""Real Go2: localize on a saved map (AMCL) and plan with Nav2. Does NOT move the robot.

  ros2 launch unitree_go2_real real_navigation.launch.py map:=$HOME/ws/maps/<name>.yaml

The robot side is real_mapping.launch.py with lio:=true and slam:=false (bridge, Point-LIO odometry,
/scan). Then map_server + AMCL, then Nav2, then RViz (the sim's Nav2 view), each stage started when
the previous one reports its nodes active, as in unitree_go2_navigation/launch/nav2.launch.py
(sim, unchanged), whose node list and velocity chain this copies.

AMCL starts at initial_x/initial_y/initial_yaw (default 0 0 0 = where the robot stood when the map's walk began) so
the map frame exists at once and Nav2 can finish starting; then correct the pose in RViz ("2D Pose Estimate") before
asking for a goal. (Without a start pose, the planner waits for the map frame forever and RViz never opens.)

Velocity path: controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed ->
collision_monitor -> /go2/nav2_cmd_vel. NOTHING subscribes to /go2/nav2_cmd_vel: Nav2 plans and
computes velocities, the robot is not commanded. Driving needs a separate bridge, started on purpose.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    real = get_package_share_directory("unitree_go2_real")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    nav2_params = LaunchConfiguration("nav2_params")
    rviz_config = os.path.join(get_package_share_directory("unitree_go2_navigation"), "rviz", "nav2_go2.rviz")

    # Scoped: arguments given to an included launch file otherwise leak into this one (its rviz:=false switched off
    # this file's RViz too).
    robot = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(real, "launch", "real_mapping.launch.py")),
        launch_arguments={"slam": "false", "lio": "true", "rviz": "false"}.items(),
    )], scoped=True)

    def localization(context):
        """AMCL with a start pose (set_initial_pose true), so map -> odom exists before RViz is up."""
        y = yaml.safe_load(open(LaunchConfiguration("nav2_params").perform(context)))
        a = y["amcl"]["ros__parameters"]
        a["set_initial_pose"] = True
        a["initial_pose"] = {"x": float(LaunchConfiguration("initial_x").perform(context)),
                             "y": float(LaunchConfiguration("initial_y").perform(context)), "z": 0.0,
                             "yaw": float(LaunchConfiguration("initial_yaw").perform(context))}
        f = tempfile.NamedTemporaryFile("w", suffix="_nav2_params.yaml", delete=False)
        yaml.safe_dump(y, f)
        f.close()
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup, "launch", "localization_launch.py")),
            launch_arguments={"map": LaunchConfiguration("map"), "use_sim_time": "false",
                              "params_file": f.name}.items())]

    common = {"parameters": [nav2_params, {"use_sim_time": False}], "output": "screen"}
    to_cmd_vel_nav = [("cmd_vel", "cmd_vel_nav")]
    nav2_nodes = [
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             remappings=to_cmd_vel_nav, **common),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", **common),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             remappings=to_cmd_vel_nav, **common),
        Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", **common),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", **common),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             remappings=to_cmd_vel_nav, **common),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", **common),
        # Waypoint following (RViz Nav2 panel: Waypoint mode -> "Start Waypoint Following"; follow_waypoints action).
        Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower", **common),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen",
             parameters=[{"use_sim_time": False}, {"autostart": True},
                         {"node_names": ["controller_server", "smoother_server", "planner_server", "behavior_server",
                                         "bt_navigator", "velocity_smoother", "collision_monitor",
                                         "waypoint_follower"]}]),
    ]

    def when_active(manager, actions):
        """Start `actions` once, when lifecycle manager `manager` reports its nodes active. Process output arrives in
        arbitrary chunks (a log line can be split), so the recent output of each process is kept and searched."""
        fired, recent = [], {}

        def handler(event):
            if fired:
                return None
            buf = (recent.get(event.pid, "") + event.text.decode(errors="ignore"))[-4000:]
            recent[event.pid] = buf
            for line in buf.splitlines():
                if f"[{manager}]" in line and "Managed nodes are active" in line:
                    fired.append(True)
                    return actions
            return None
        return RegisterEventHandler(OnProcessIO(on_stdout=handler, on_stderr=handler))

    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": False}], output="screen", condition=IfCondition(LaunchConfiguration("rviz")))

    return LaunchDescription([
        DeclareLaunchArgument("map", description="Saved map yaml, e.g. ~/ws/maps/lab_dense.yaml"),
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz with the Nav2 view"),
        DeclareLaunchArgument("nav2_params", default_value=os.path.join(real, "config", "nav2_real.yaml"),
                              description="Nav2 + AMCL parameter file"),
        DeclareLaunchArgument("initial_x", default_value="0.0", description="AMCL start pose on the map (m)"),
        DeclareLaunchArgument("initial_y", default_value="0.0", description="AMCL start pose on the map (m)"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0", description="AMCL start heading on the map (rad)"),
        DeclareLaunchArgument("localization_delay", default_value="8.0",
                              description="Seconds before map_server + AMCL start (Point-LIO and /scan come up first)"),
        robot,
        TimerAction(period=LaunchConfiguration("localization_delay"), actions=[OpaqueFunction(function=localization)]),
        when_active("lifecycle_manager_localization", nav2_nodes),
        when_active("lifecycle_manager_navigation", [rviz]),
    ])
