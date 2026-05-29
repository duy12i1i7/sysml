import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration("model")
    rviz = LaunchConfiguration("rviz")
    gz_args = LaunchConfiguration("gz_args")
    config_file = LaunchConfiguration("config_file")
    output_dir = LaunchConfiguration("output_dir")
    use_target_publisher = LaunchConfiguration("use_target_publisher")
    use_rviz_target = LaunchConfiguration("use_rviz_target")
    use_metrics = LaunchConfiguration("use_metrics")

    tb4_gz = get_package_share_directory("turtlebot4_gz_bringup")
    tb4_gui = get_package_share_directory("turtlebot4_gz_gui_plugins")
    tb4_desc = get_package_share_directory("turtlebot4_description")
    create_desc = get_package_share_directory("irobot_create_description")
    create_gz = get_package_share_directory("irobot_create_gz_bringup")
    create_plugins = get_package_share_directory("irobot_create_gz_plugins")

    gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    spawn_launch = PathJoinSubstitution(
        [FindPackageShare("turtlebot4_gz_bringup"), "launch", "turtlebot4_spawn.launch.py"]
    )
    default_config = PathJoinSubstitution(
        [FindPackageShare("paper_tb4_swarm"), "config", "official_tb4_sim.yaml"]
    )

    def spawn(namespace: str, x: str, y: str):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                "namespace": namespace,
                "model": model,
                "rviz": rviz,
                "x": x,
                "y": y,
                "yaw": "0.0",
                "nav2": "false",
                "slam": "false",
                "localization": "false",
            }.items(),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="lite", choices=["standard", "lite"]),
            DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("output_dir", default_value="metrics/official_tb4_sim"),
            DeclareLaunchArgument("use_target_publisher", default_value="true"),
            DeclareLaunchArgument("use_rviz_target", default_value="false"),
            DeclareLaunchArgument("use_metrics", default_value="true"),
            DeclareLaunchArgument(
                "gz_args",
                default_value="warehouse.sdf -r -s -v 1",
                description=(
                    "Server-only by default to avoid Gazebo GUI/EGL crashes. "
                    "Use 'warehouse.sdf -r -v 2' only if render is stable."
                ),
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=":".join(
                    [
                        os.path.join(tb4_gz, "worlds"),
                        os.path.join(create_gz, "worlds"),
                        str(Path(tb4_desc).parent.resolve()),
                        str(Path(create_desc).parent.resolve()),
                    ]
                ),
            ),
            SetEnvironmentVariable(
                name="GZ_GUI_PLUGIN_PATH",
                value=":".join(
                    [
                        os.path.join(tb4_gui, "lib"),
                        os.path.join(create_plugins, "lib"),
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={"gz_args": gz_args}.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="clock_bridge",
                output="screen",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            ),
            spawn("robot1", "0.0", "0.0"),
            spawn("robot2", "0.0", "1.0"),
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
