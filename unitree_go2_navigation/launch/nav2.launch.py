"""Navigation: pointcloud_to_laserscan + AMCL on a saved map + Nav2 + RViz with the Nav2 view.

Start the simulator first, in its own terminal (unitree_go2_sim unitree_go2_launch.py),
wait until it has settled (about 90 s), then launch this. The robot spawns at the map
origin, which is where AMCL is seeded (config/nav2_params.yaml).

The nodes and their order are the committed, tested ones (AMCL and Nav2 brought up
together after a delay); only the simulator include and the SLAM mode were taken out.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    # The scan conversion starts first so AMCL and the costmaps find /scan and a
    # complete TF tree when they come up.
    declare_nav2_delay = DeclareLaunchArgument(
        "nav2_delay",
        default_value="5.0",
        description="Seconds to wait before bringing up AMCL and Nav2",
    )
    # Starting RViz while Nav2's nodes are still being discovered has saturated DDS
    # on this machine (/clock stops reaching ROS, Nav2 stalls). In the tested runs RViz
    # was started by hand after Nav2 was active, so it opens late here too.
    declare_rviz_delay = DeclareLaunchArgument(
        "rviz_delay",
        default_value="25.0",
        description="Seconds to wait before opening RViz, after Nav2 is up",
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
            declare_nav2_delay,
            declare_rviz_delay,
            pointcloud_to_laserscan_node,
            unmapped_obstacle_marker,
            TimerAction(
                period=LaunchConfiguration("nav2_delay"),
                actions=[localization] + nav2_nodes,
            ),
            TimerAction(period=LaunchConfiguration("rviz_delay"), actions=[rviz_node]),
        ]
    )
