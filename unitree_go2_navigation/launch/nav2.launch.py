"""Navigation: pointcloud_to_laserscan + AMCL on a saved map + Nav2 + RViz with the Nav2 view.

Start the simulator first, in its own terminal (unitree_go2_sim unitree_go2_launch.py),
wait until it has settled (about 90 s), then launch this. The robot spawns at the map
origin, which is where AMCL is seeded (config/nav2_params.yaml).

The nodes and parameters are the committed, tested ones; the simulator include and the
SLAM mode were taken out. AMCL comes up first, then Nav2, then RViz, each started when
the previous stage reports its nodes active.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessIO
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    unitree_go2_navigation = get_package_share_directory("unitree_go2_navigation")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    pointcloud_to_laserscan_config = os.path.join(
        unitree_go2_navigation, "config", "pointcloud_to_laserscan.yaml"
    )
    nav2_params = os.path.join(unitree_go2_navigation, "config", "nav2_params.yaml")
    default_map = os.path.join(unitree_go2_navigation, "maps", "maze_world2.yaml")
    rviz_config = os.path.join(unitree_go2_navigation, "rviz", "nav2_go2.rviz")

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Map yaml to localise against. It must be a map of the world the "
        "simulator is running (maze_world2 is a map of maze.sdf)",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Open RViz with the Nav2 view"
    )
    # The scan conversion starts first so AMCL finds /scan when it comes up. After that,
    # each stage starts when the previous one reports "Managed nodes are active":
    # AMCL + map_server -> Nav2 -> RViz. Starting the two lifecycle managers at the same
    # instant occasionally dropped a lifecycle reply and stalled Nav2 half-configured,
    # and starting RViz during Nav2's discovery is another known stall trigger. If a
    # stage stalls, the next one (and RViz) never starts: relaunch.
    declare_localization_delay = DeclareLaunchArgument(
        "localization_delay",
        default_value="3.0",
        description="Seconds to wait before bringing up map_server and AMCL",
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

    # Gives obstacles that are not in the saved map an assumed size for the global
    # planner, so gaps too narrow for the robot read as closed and it detours.
    unmapped_obstacle_marker = Node(
        package="unitree_go2_navigation",
        executable="unmapped_obstacle_marker.py",
        name="unmapped_obstacle_marker",
        output="screen",
        parameters=[{"use_sim_time": True}],
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

    def when_active(manager, actions):
        """Start `actions` once, when lifecycle manager `manager` reports its nodes active."""
        fired = []

        def handler(event):
            if fired:
                return None
            text = event.text.decode(errors="ignore")
            if f"[{manager}]" in text and "Managed nodes are active" in text:
                fired.append(True)
                return actions
            return None

        return RegisterEventHandler(OnProcessIO(on_stdout=handler, on_stderr=handler))

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
            declare_map,
            declare_rviz,
            declare_localization_delay,
            pointcloud_to_laserscan_node,
            unmapped_obstacle_marker,
            TimerAction(
                period=LaunchConfiguration("localization_delay"),
                actions=[localization],
            ),
            when_active("lifecycle_manager_localization", nav2_nodes),
            when_active("lifecycle_manager_navigation", [rviz_node]),
        ]
    )
