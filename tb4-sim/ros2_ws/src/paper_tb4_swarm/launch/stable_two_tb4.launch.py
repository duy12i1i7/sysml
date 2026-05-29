from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _bridge(namespace: str) -> Node:
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{namespace}_bridge",
        output="screen",
        arguments=[
            f"/{namespace}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            f"/{namespace}/odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
    )


def generate_launch_description():
    use_gazebo = LaunchConfiguration("use_gazebo")
    gz_args = LaunchConfiguration("gz_args")
    config_file = LaunchConfiguration("config_file")
    map_file = LaunchConfiguration("map_file")
    output_dir = LaunchConfiguration("output_dir")
    use_target_publisher = LaunchConfiguration("use_target_publisher")
    use_rviz_target = LaunchConfiguration("use_rviz_target")
    use_metrics = LaunchConfiguration("use_metrics")
    use_map_markers = LaunchConfiguration("use_map_markers")
    target_mode = LaunchConfiguration("target_mode")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")

    default_world = PathJoinSubstitution(
        [FindPackageShare("paper_tb4_swarm"), "worlds", "two_tb4_lab.sdf"]
    )
    default_config = PathJoinSubstitution(
        [FindPackageShare("paper_tb4_swarm"), "config", "stable_sim.yaml"]
    )
    default_map = PathJoinSubstitution(
        [FindPackageShare("paper_tb4_swarm"), "maps", "lab.yaml"]
    )
    gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gazebo", default_value="true"),
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("map_file", default_value=default_map),
            DeclareLaunchArgument("output_dir", default_value="metrics/stable_sim"),
            DeclareLaunchArgument("use_target_publisher", default_value="true"),
            DeclareLaunchArgument("use_rviz_target", default_value="false"),
            DeclareLaunchArgument("use_metrics", default_value="true"),
            DeclareLaunchArgument("use_map_markers", default_value="true"),
            DeclareLaunchArgument("target_mode", default_value="fixed"),
            DeclareLaunchArgument("target_x", default_value="1.722"),
            DeclareLaunchArgument("target_y", default_value="0.717"),
            DeclareLaunchArgument("gz_args", default_value=[default_world, " -r -s -v 1"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                condition=IfCondition(use_gazebo),
                launch_arguments={"gz_args": gz_args}.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="clock_bridge",
                output="screen",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                condition=IfCondition(use_gazebo),
            ),
            _bridge("robot1"),
            _bridge("robot2"),
            Node(
                package="paper_tb4_swarm",
                executable="odom_offsetter",
                name="odom_offsetter",
                output="screen",
                parameters=[config_file],
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
                executable="target_publisher",
                name="target_publisher",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "mode": target_mode,
                        "target_x": ParameterValue(target_x, value_type=float),
                        "target_y": ParameterValue(target_y, value_type=float),
                    },
                ],
                condition=IfCondition(use_target_publisher),
            ),
            Node(
                package="paper_tb4_swarm",
                executable="static_map_publisher",
                name="static_map_publisher",
                output="screen",
                parameters=[
                    {
                        "map_yaml": map_file,
                        "map_topic": "/map",
                    }
                ],
            ),
            Node(
                package="paper_tb4_swarm",
                executable="occupancy_grid_marker",
                name="occupancy_grid_marker",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(use_map_markers),
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
                parameters=[config_file, {"map_yaml": map_file}],
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
