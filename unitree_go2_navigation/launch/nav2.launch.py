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
    unitree_go2_description = get_package_share_directory("unitree_go2_description")
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
    default_world = os.path.join(unitree_go2_description, "worlds", "default.sdf")

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
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world sdf path to forward to the sim",
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use the Gazebo gui"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    # 40 s, not 25: Gazebo is still loading controllers at 25 s, and the CPU
    # contention made Nav2's lifecycle service calls time out. The nodes
    # configured fine but their replies were dropped, so lifecycle_manager
    # reported "Failed to bring up all requested nodes" and aborted.
    declare_nav2_delay = DeclareLaunchArgument(
        "nav2_delay",
        default_value="40.0",
        description="Seconds to wait for Gazebo to spawn the robot and settle "
        "before bringing up Nav2, so the costmaps find a complete TF tree and "
        "the lifecycle service calls are not starved",
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

    # Gives obstacles that are not in the saved map an assumed size for the global
    # planner. Localization only: in SLAM mode the map is still being built.
    unmapped_obstacle_marker = Node(
        package="unitree_go2_navigation",
        executable="unmapped_obstacle_marker.py",
        name="unmapped_obstacle_marker",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(is_slam),
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
            declare_world,
            declare_gui,
            declare_rviz,
            declare_nav2_delay,
            sim,
            pointcloud_to_laserscan_node,
            unmapped_obstacle_marker,
            slam_toolbox_node,
            configure_slam_toolbox,
            activate_slam_toolbox,
            TimerAction(
                period=LaunchConfiguration("nav2_delay"),
                actions=[localization] + nav2_nodes,
            ),
        ]
    )
