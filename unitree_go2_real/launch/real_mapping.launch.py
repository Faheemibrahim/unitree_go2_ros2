"""Real Go2: official robot model + TF + lidar -> /scan -> slam_toolbox + RViz.

The robot side of unitree_go2_navigation's slam.launch.py (sim files untouched). Start
the container with `~/go2_docker/run.sh real`, then:

  ros2 launch unitree_go2_real real_mapping.launch.py            # mapping
  ros2 launch unitree_go2_real real_mapping.launch.py slam:=false  # model, TF, cloud only
  ros2 launch unitree_go2_real real_mapping.launch.py lio:=true   # x, y, yaw from Point-LIO
  ros2 launch unitree_go2_real real_mapping.launch.py overrides:=<file.yaml>  # change one value
    (see config/no_overrides.yaml; applied after the normal config of each node)

Listen-only: nothing here sends commands to the robot. Drive with the Unitree remote.
Save the map with `ros2 run nav2_map_server map_saver_cli -f <name>`.

  /utlidar/robot_odom, /utlidar/cloud_deskewed -> real_tf_bridge (TF, PC-time stamps)
  /lowstate -> lowstate_joint_states -> /joint_states -> robot_state_publisher (go2_description)
  /go2/cloud_deskewed -> body_crop -> cloud_accumulator -> pointcloud_to_laserscan -> /scan
  /scan -> slam_toolbox -> /map, map -> odom
  lio:=true  /utlidar/{cloud_deskewed,imu,robot_odom} -> lio_input -> Point-LIO (point_lio_unilidar,
             config/point_lio_go2.yaml) -> /go2/lio_odom -> real_tf_bridge (pose_source lio)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    real = get_package_share_directory("unitree_go2_real")
    scan_config = os.path.join(real, "config", "pointcloud_to_laserscan.yaml")
    bridge_config = os.path.join(real, "config", "real_bridge.yaml")
    slam_config = LaunchConfiguration("slam_config")
    rviz_config = os.path.join(real, "rviz", "real_mapping.rviz")

    # The official model's root link is `base`; the robot's odometry uses `base_link`.
    urdf_file = os.path.join(
        get_package_share_directory("go2_description"), "urdf", "go2_description.urdf"
    )
    with open(urdf_file) as f:
        urdf = f.read().replace(
            "</robot>",
            '<link name="base_link"/><joint name="base_link_to_base" type="fixed">'
            '<parent link="base_link"/><child link="base"/></joint></robot>',
        )

    slam = LaunchConfiguration("slam")
    overrides = LaunchConfiguration("overrides")
    rviz = LaunchConfiguration("rviz")
    no_sim_time = {"use_sim_time": False}

    lio = LaunchConfiguration("lio")
    bridge = Node(
        package="unitree_go2_real", executable="real_tf_bridge.py",
        name="real_tf_bridge", output="screen",
        parameters=[bridge_config,
                    {"pose_source": PythonExpression(["'lio' if '", lio, "' == 'true' else 'robot'"])},
                    overrides],
    )
    lio_input = Node(
        package="unitree_go2_real", executable="lio_input.py", name="lio_input", output="screen",
        parameters=[overrides], condition=IfCondition(lio),
    )
    point_lio = Node(
        package="point_lio_unilidar", executable="pointlio_mapping", name="point_lio", output="screen",
        parameters=[os.path.join(real, "config", "point_lio_go2.yaml")],
        remappings=[("/aft_mapped_to_init", "/go2/lio_odom"), ("/cloud_registered", "/go2/lio_cloud_registered")],
        condition=IfCondition(lio),
    )
    joint_states = Node(
        package="unitree_go2_real", executable="lowstate_joint_states.py",
        name="lowstate_joint_states", output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": urdf}, no_sim_time],
    )

    body_crop = Node(
        package="pcl_ros", executable="filter_crop_box_node", name="body_crop",
        output="screen", parameters=[scan_config, overrides],
        remappings=[("input", "/go2/cloud_deskewed"), ("output", "/l1_cloud_cropped")],
    )
    # Keep only the scan band before merging: ~99% of a real message is floor, and merging
    # whole clouds made 1.8 MB messages that were dropped in transport (merged topic 5.6 Hz,
    # gaps to 2.3 s, against 15 Hz in). z in odom is height above the floor.
    band = Node(
        package="pcl_ros", executable="filter_passthrough_node", name="band",
        output="screen", parameters=[scan_config, overrides],
        remappings=[("input", "/l1_cloud_cropped"), ("output", "/l1_cloud_band")],
    )
    # Reused unchanged from the sim package: it only joins message bytes.
    cloud_accumulator = Node(
        package="unitree_go2_navigation", executable="cloud_accumulator.py",
        name="cloud_accumulator", output="screen", parameters=[scan_config, overrides],
        remappings=[("input", "/l1_cloud_band"), ("output", "/l1_cloud_merged")],
    )
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan", output="screen", parameters=[scan_config, overrides],
        remappings=[("cloud_in", "/l1_cloud_merged"), ("scan", "/scan")],
    )

    slam_toolbox = LifecycleNode(
        package="slam_toolbox", executable="async_slam_toolbox_node",
        name="slam_toolbox", namespace="", output="screen",
        parameters=[slam_config, {"use_lifecycle_manager": False}, overrides],
    )
    configure_slam = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_toolbox),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate_slam = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_toolbox, start_state="configuring", goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))

    # With SLAM, RViz opens once slam_toolbox is active, so `map` exists when it starts.
    rviz_mapping = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_config], parameters=[no_sim_time], condition=IfCondition(rviz),
    )
    open_rviz_when_active = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_toolbox, goal_state="active", entities=[rviz_mapping],
    ))
    # Without SLAM there is no `map` frame: view in odom.
    rviz_no_slam = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_config, "-f", "odom"], parameters=[no_sim_time],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("slam", default_value="true", description="Run slam_toolbox"),
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz"),
        DeclareLaunchArgument("lio", default_value="false",
                              description="x, y, yaw from Point-LIO (needs point_lio_unilidar built)"),
        DeclareLaunchArgument(
            "slam_config",
            default_value=PythonExpression([
                "'", os.path.join(real, "config", "slam_lio.yaml"), "' if '", LaunchConfiguration("lio"),
                "' == 'true' else '", os.path.join(real, "config", "slam_toolbox.yaml"), "'"]),
            description="slam_toolbox parameter file (default: config/slam_lio.yaml with lio:=true, else "
                        "config/slam_toolbox.yaml; config/slam_stock.yaml = slam_toolbox's own defaults)"),
        DeclareLaunchArgument(
            "overrides", default_value=os.path.join(real, "config", "no_overrides.yaml"),
            description="Parameter file applied after each node's config (one run, one change)"),
        bridge,
        lio_input,
        point_lio,
        joint_states,
        robot_state_publisher,
        body_crop,
        band,
        cloud_accumulator,
        pointcloud_to_laserscan,
        GroupAction(
            [slam_toolbox, configure_slam, activate_slam, open_rviz_when_active],
            condition=IfCondition(slam),
        ),
        GroupAction([rviz_no_slam], condition=UnlessCondition(slam)),
    ])
