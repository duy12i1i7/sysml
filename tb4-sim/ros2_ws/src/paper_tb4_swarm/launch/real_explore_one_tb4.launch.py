from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap


ARGUMENTS = [
    DeclareLaunchArgument("namespace", default_value="/robot1"),
    DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
    DeclareLaunchArgument("start_slam", default_value="true", choices=["true", "false"]),
    DeclareLaunchArgument("start_nav2", default_value="true", choices=["true", "false"]),
    DeclareLaunchArgument("relay_cmd_vel", default_value="true", choices=["true", "false"]),
    DeclareLaunchArgument("obstacle_clearance_m", default_value="0.42"),
    DeclareLaunchArgument("min_frontier_distance_m", default_value="0.65"),
    DeclareLaunchArgument("min_frontier_size_cells", default_value="8"),
    DeclareLaunchArgument("approach_offset_m", default_value="0.25"),
    DeclareLaunchArgument("goal_timeout_sec", default_value="90.0"),
    DeclareLaunchArgument("slam_params_file", default_value=PathJoinSubstitution([
        get_package_share_directory("paper_tb4_swarm"),
        "config",
        "real_explore_slam.yaml",
    ])),
    DeclareLaunchArgument("nav2_params_file", default_value=PathJoinSubstitution([
        get_package_share_directory("paper_tb4_swarm"),
        "config",
        "real_explore_nav2.yaml",
    ])),
]


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace")
    namespace_str = namespace.perform(context)
    if namespace_str and not namespace_str.startswith("/"):
        namespace_str = "/" + namespace_str

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_slam = LaunchConfiguration("start_slam")
    start_nav2 = LaunchConfiguration("start_nav2")
    relay_cmd_vel = LaunchConfiguration("relay_cmd_vel")
    obstacle_clearance_m = LaunchConfiguration("obstacle_clearance_m")
    min_frontier_distance_m = LaunchConfiguration("min_frontier_distance_m")
    min_frontier_size_cells = LaunchConfiguration("min_frontier_size_cells")
    approach_offset_m = LaunchConfiguration("approach_offset_m")
    goal_timeout_sec = LaunchConfiguration("goal_timeout_sec")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    tb4_navigation = get_package_share_directory("turtlebot4_navigation")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb4_navigation, "launch", "slam.launch.py"])
        ),
        launch_arguments=[
            ("namespace", namespace_str),
            ("use_sim_time", use_sim_time),
            ("sync", "false"),
            ("params", slam_params_file),
        ],
        condition=IfCondition(start_slam),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb4_navigation, "launch", "nav2.launch.py"])
        ),
        launch_arguments=[
            ("namespace", namespace_str),
            ("use_sim_time", use_sim_time),
            ("params_file", nav2_params_file),
        ],
        condition=IfCondition(start_nav2),
    )

    tf_group = GroupAction([
        PushRosNamespace(namespace_str),
        SetRemap("/tf", namespace_str + "/tf"),
        SetRemap("/tf_static", namespace_str + "/tf_static"),
        Node(
            package="paper_tb4_swarm",
            executable="odom_tf_broadcaster",
            name="odom_tf_broadcaster",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
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
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])

    explorer_group = GroupAction([
        PushRosNamespace(namespace_str),
        SetRemap("/tf", namespace_str + "/tf"),
        SetRemap("/tf_static", namespace_str + "/tf_static"),
        Node(
            package="paper_tb4_swarm",
            executable="frontier_explorer",
            name="frontier_explorer",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "obstacle_clearance_m": obstacle_clearance_m,
                "min_frontier_distance_m": min_frontier_distance_m,
                "min_frontier_size_cells": min_frontier_size_cells,
                "approach_offset_m": approach_offset_m,
                "goal_timeout_sec": goal_timeout_sec,
            }],
        ),
        Node(
            package="paper_tb4_swarm",
            executable="cmd_vel_relay",
            name="cmd_vel_relay",
            output="screen",
            condition=IfCondition(relay_cmd_vel),
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])

    delayed_nav2 = TimerAction(period=5.0, actions=[nav2])
    delayed_explorer = TimerAction(period=8.0, actions=[explorer_group])

    return [slam, tf_group, delayed_nav2, delayed_explorer]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
