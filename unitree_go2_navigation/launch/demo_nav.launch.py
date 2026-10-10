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
    unitree_go2_description = get_package_share_directory("unitree_go2_description")
    unitree_go2_navigation = get_package_share_directory("unitree_go2_navigation")

    default_map = os.path.join(unitree_go2_navigation, "maps", "default_world.yaml")
    default_world = os.path.join(unitree_go2_description, "worlds", "default.sdf")
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
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world sdf path to forward to the sim",
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use the Gazebo gui"
    )
    # Off by default. Starting RViz from this launch file saturates DDS
    # discovery on this setup: /clock stops being delivered to ROS even though
    # Gazebo keeps running, and Nav2 never activates. A delay makes it less
    # likely but not reliable - it was seen to work once and fail on the next
    # run with the same timing. Starting RViz by hand, after the stack is up,
    # was reliable, so that is what the README and the setup guide tell you to
    # do. Set rviz:=true to start it from here anyway.
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz with the Nav2 view"
    )
    declare_rviz_delay = DeclareLaunchArgument(
        "rviz_delay",
        default_value="45.0",
        description="Seconds before RViz starts, when rviz:=true",
    )
    declare_nav2_delay = DeclareLaunchArgument(
        "nav2_delay",
        default_value="40.0",
        description="Seconds to wait for Gazebo to spawn the robot and settle "
        "before Nav2 starts",
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
            "world": LaunchConfiguration("world"),
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
            declare_world,
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
