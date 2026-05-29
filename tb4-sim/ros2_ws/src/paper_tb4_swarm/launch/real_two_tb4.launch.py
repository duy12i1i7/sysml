from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    output_dir = LaunchConfiguration("output_dir")
    use_target_publisher = LaunchConfiguration("use_target_publisher")
    use_rviz_target = LaunchConfiguration("use_rviz_target")
    use_cmd_vel_follower = LaunchConfiguration("use_cmd_vel_follower")
    use_metrics = LaunchConfiguration("use_metrics")

    default_config = PathJoinSubstitution(
        [FindPackageShare("paper_tb4_swarm"), "config", "real_two_tb4.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("output_dir", default_value="metrics/real_two_tb4"),
            DeclareLaunchArgument(
                "use_target_publisher",
                default_value="false",
                description="Publish a fixed target. Leave false when targets come from an operator or perception.",
            ),
            DeclareLaunchArgument(
                "use_rviz_target",
                default_value="false",
                description="Accept /goal_pose or /clicked_point from RViz and publish /target_pose.",
            ),
            DeclareLaunchArgument(
                "use_cmd_vel_follower",
                default_value="false",
                description="Directly command /robot*/cmd_vel_unstamped. Use only in a controlled lab test.",
            ),
            DeclareLaunchArgument("use_metrics", default_value="true"),
            Node(
                package="paper_tb4_swarm",
                executable="coordinator",
                name="paper_swarm_coordinator",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="paper_tb4_swarm",
                executable="target_publisher",
                name="target_publisher",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(use_target_publisher),
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
                executable="simple_goal_follower",
                name="simple_goal_follower",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(use_cmd_vel_follower),
            ),
            Node(
                package="paper_tb4_swarm",
                executable="metrics_logger",
                name="metrics_logger",
                output="screen",
                parameters=[config_file, {"output_dir": output_dir}],
                condition=IfCondition(use_metrics),
            ),
        ]
    )
