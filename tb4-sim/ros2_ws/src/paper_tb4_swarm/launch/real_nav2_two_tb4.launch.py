from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _robot_tf_group(namespace: str) -> GroupAction:
    ns = namespace.strip("/")
    return GroupAction([
        PushRosNamespace(ns),
        SetRemap("/tf", f"/{ns}/tf"),
        SetRemap("/tf_static", f"/{ns}/tf_static"),
        Node(
            package="paper_tb4_swarm",
            executable="odom_tf_broadcaster",
            name="odom_tf_broadcaster",
            output="screen",
            parameters=[
                {
                    "odom_topic": f"/{ns}/odom",
                    "parent_frame": "odom",
                    "child_frame": "base_link",
                }
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rplidar_static_transform",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.22",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "rplidar_link",
            ],
        ),
    ])


def _robot_nav2_group(
    namespace: str,
    map_file,
    include_localization: bool = True,
    include_navigation: bool = True,
) -> GroupAction:
    ns = namespace.strip("/")
    nav2_params = PathJoinSubstitution(
        [FindPackageShare("turtlebot4_navigation"), "config", "nav2.yaml"]
    )
    localization_params = PathJoinSubstitution(
        [FindPackageShare("turtlebot4_navigation"), "config", "localization.yaml"]
    )
    configured_nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            root_key=ns,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    configured_localization_params = ParameterFile(
        RewrittenYaml(
            source_file=localization_params,
            root_key=ns,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    common_remaps = [
        ("/tf", f"/{ns}/tf"),
        ("/tf_static", f"/{ns}/tf_static"),
        (f"/{ns}/global_costmap/scan", f"/{ns}/scan"),
        (f"/{ns}/local_costmap/scan", f"/{ns}/scan"),
    ]

    actions = []

    if include_localization:
        actions.extend(
            [
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    namespace=ns,
                    output="screen",
                    parameters=[
                        configured_localization_params,
                        {"yaml_filename": map_file, "use_sim_time": False},
                    ],
                    arguments=["--ros-args", "--log-level", "info"],
                    remappings=common_remaps,
                ),
                Node(
                    package="nav2_amcl",
                    executable="amcl",
                    name="amcl",
                    namespace=ns,
                    output="screen",
                    parameters=[configured_localization_params, {"use_sim_time": False}],
                    arguments=["--ros-args", "--log-level", "info"],
                    remappings=common_remaps,
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_localization",
                    namespace=ns,
                    output="screen",
                    parameters=[
                        {"autostart": True, "node_names": ["map_server", "amcl"]},
                        {"use_sim_time": False},
                    ],
                    arguments=["--ros-args", "--log-level", "info"],
                ),
            ]
        )

    if include_navigation:
        actions.extend(
            [
                Node(
                    package="nav2_controller",
                    executable="controller_server",
                    namespace=ns,
                    output="screen",
                    parameters=[configured_nav2_params, {"use_sim_time": False}],
                    arguments=["--ros-args", "--log-level", "info"],
                    remappings=common_remaps,
                ),
                Node(
                    package="nav2_planner",
                    executable="planner_server",
                    name="planner_server",
                    namespace=ns,
                    output="screen",
                    parameters=[configured_nav2_params, {"use_sim_time": False}],
                    arguments=["--ros-args", "--log-level", "info"],
                    remappings=common_remaps,
                ),
                Node(
                    package="nav2_behaviors",
                    executable="behavior_server",
                    name="behavior_server",
                    namespace=ns,
                    output="screen",
                    parameters=[configured_nav2_params, {"use_sim_time": False}],
                    arguments=["--ros-args", "--log-level", "info"],
                    remappings=common_remaps,
                ),
                Node(
                    package="nav2_bt_navigator",
                    executable="bt_navigator",
                    name="bt_navigator",
                    namespace=ns,
                    output="screen",
                    parameters=[configured_nav2_params, {"use_sim_time": False}],
                    arguments=["--ros-args", "--log-level", "info"],
                    remappings=common_remaps,
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    namespace=ns,
                    output="screen",
                    parameters=[
                        {
                            "autostart": True,
                            "node_names": [
                                "controller_server",
                                "planner_server",
                                "behavior_server",
                                "bt_navigator",
                            ],
                        },
                        {"use_sim_time": False},
                    ],
                    arguments=["--ros-args", "--log-level", "info"],
                ),
            ]
        )

    return GroupAction(actions)


def _global_localization(namespace: str) -> ExecuteProcess:
    return ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            f"/{namespace.strip('/')}/reinitialize_global_localization",
            "std_srvs/srv/Empty",
            "{}",
        ],
        output="screen",
    )


def generate_launch_description():
    map_file = LaunchConfiguration("map")
    config_file = LaunchConfiguration("config_file")
    output_dir = LaunchConfiguration("output_dir")
    use_navigation_stack = LaunchConfiguration("use_navigation_stack")
    use_robot2_navigation_stack = LaunchConfiguration("use_robot2_navigation_stack")
    use_follower_controller = LaunchConfiguration("use_follower_controller")
    use_rviz_target = LaunchConfiguration("use_rviz_target")
    use_metrics = LaunchConfiguration("use_metrics")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay")
    global_localization_delay = LaunchConfiguration("global_localization_delay")
    navigation_start_delay = LaunchConfiguration("navigation_start_delay")
    use_auto_global_localization = LaunchConfiguration("use_auto_global_localization")

    default_config = PathJoinSubstitution(
        [FindPackageShare("paper_tb4_swarm"), "config", "real_nav2_two_tb4.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("output_dir", default_value="metrics/real_nav2_two_tb4"),
            DeclareLaunchArgument("use_navigation_stack", default_value="true"),
            DeclareLaunchArgument("use_robot2_navigation_stack", default_value="false"),
            DeclareLaunchArgument("use_follower_controller", default_value="false"),
            DeclareLaunchArgument("use_rviz_target", default_value="true"),
            DeclareLaunchArgument("use_metrics", default_value="true"),
            DeclareLaunchArgument("nav2_start_delay", default_value="6.0"),
            DeclareLaunchArgument("global_localization_delay", default_value="18.0"),
            DeclareLaunchArgument("navigation_start_delay", default_value="45.0"),
            DeclareLaunchArgument("use_auto_global_localization", default_value="true"),
            _robot_tf_group("/robot1"),
            _robot_tf_group("/robot2"),
            TimerAction(
                period=nav2_start_delay,
                actions=[
                    GroupAction(
                        [_robot_nav2_group("/robot1", map_file, True, False)],
                        condition=IfCondition(use_navigation_stack),
                    ),
                    _robot_nav2_group("/robot2", map_file, True, False),
                ],
            ),
            TimerAction(
                period=global_localization_delay,
                actions=[
                    GroupAction(
                        [_global_localization("/robot1"), _global_localization("/robot2")],
                        condition=IfCondition(use_auto_global_localization),
                    ),
                ],
            ),
            TimerAction(
                period=navigation_start_delay,
                actions=[
                    GroupAction(
                        [_robot_nav2_group("/robot1", map_file, False, True)],
                        condition=IfCondition(use_navigation_stack),
                    ),
                    GroupAction(
                        [_robot_nav2_group("/robot2", map_file, False, True)],
                        condition=IfCondition(use_robot2_navigation_stack),
                    ),
                ],
            ),
            Node(
                package="paper_tb4_swarm",
                executable="coordinator",
                name="paper_swarm_coordinator",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="paper_tb4_swarm",
                executable="rviz_target_bridge",
                name="rviz_target_bridge",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(use_rviz_target),
            ),
            Node(
                package="paper_tb4_swarm",
                executable="nav2_goal_dispatcher",
                name="nav2_goal_dispatcher",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="paper_tb4_swarm",
                executable="simple_goal_follower",
                name="simple_goal_follower",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(use_follower_controller),
            ),
            Node(
                package="paper_tb4_swarm",
                executable="metrics_logger",
                name="metrics_logger",
                output="screen",
                parameters=[config_file, {"output_dir": output_dir}],
                condition=IfCondition(use_metrics),
            ),
            Node(
                package="paper_tb4_swarm",
                executable="occupancy_grid_marker",
                name="occupancy_grid_marker",
                output="screen",
                parameters=[
                    {
                        "map_topic": "/robot1/map",
                        "marker_topic": "/swarm/map_markers",
                        "goal_min_clearance_m": 0.25,
                        "unknown_blocks_goal": True,
                    }
                ],
                condition=IfCondition(use_rviz_target),
            ),
        ]
    )
