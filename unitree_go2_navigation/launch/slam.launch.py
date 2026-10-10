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
    unitree_go2_description = get_package_share_directory("unitree_go2_description")
    unitree_go2_navigation = get_package_share_directory("unitree_go2_navigation")

    pointcloud_to_laserscan_config = os.path.join(
        unitree_go2_navigation, "config", "pointcloud_to_laserscan.yaml"
    )
    slam_toolbox_config = os.path.join(
        unitree_go2_navigation, "config", "slam_toolbox.yaml"
    )
    default_world = os.path.join(unitree_go2_description, "worlds", "default.sdf")

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world sdf path to forward to the sim",
    )

    # slam_toolbox owns map->odom here, so the sim must not also publish it.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unitree_go2_sim, "launch", "unitree_go2_launch.py")
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "world": LaunchConfiguration("world"),
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
            declare_world,
            sim,
            pointcloud_to_laserscan_node,
            slam_toolbox_node,
            configure_slam_toolbox,
            activate_slam_toolbox,
        ]
    )
