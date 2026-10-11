"""Mapping: lidar to /scan (scan.launch.py) + slam_toolbox + RViz with a mapping view.

Start the simulator first, in its own terminal (unitree_go2_sim unitree_go2_launch.py),
let it settle, then launch this and drive with teleop. Save the map with
`ros2 run nav2_map_server map_saver_cli -f <name>`.
"""

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
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    unitree_go2_navigation = get_package_share_directory("unitree_go2_navigation")

    slam_toolbox_config = os.path.join(
        unitree_go2_navigation, "config", "slam_toolbox.yaml"
    )
    rviz_config = os.path.join(unitree_go2_navigation, "rviz", "slam_go2.rviz")

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Open RViz with the mapping view"
    )

    scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unitree_go2_navigation, "launch", "scan.launch.py")
        )
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # RViz opens once slam_toolbox is active, so the map and map->odom exist when it starts.
    open_rviz_when_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state="active",
            entities=[rviz_node],
        )
    )

    return LaunchDescription(
        [
            declare_rviz,
            scan,
            slam_toolbox_node,
            configure_slam_toolbox,
            activate_slam_toolbox,
            open_rviz_when_active,
        ]
    )
