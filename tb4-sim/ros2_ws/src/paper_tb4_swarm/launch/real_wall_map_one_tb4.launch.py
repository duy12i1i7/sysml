from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap


ARGUMENTS = [
    DeclareLaunchArgument("namespace", default_value="/robot1"),
    DeclareLaunchArgument("scan_yaw_offset_deg", default_value="0.0"),
    DeclareLaunchArgument("duration_sec", default_value="180.0"),
    DeclareLaunchArgument("initial_spin_sec", default_value="22.0"),
    DeclareLaunchArgument("target_wall_distance_m", default_value="0.65"),
    DeclareLaunchArgument("front_stop_distance_m", default_value="0.72"),
    DeclareLaunchArgument("emergency_stop_distance_m", default_value="0.34"),
    DeclareLaunchArgument("linear_speed_mps", default_value="0.055"),
    DeclareLaunchArgument("turn_speed_radps", default_value="0.34"),
]


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace")
    namespace_str = namespace.perform(context)
    if namespace_str and not namespace_str.startswith("/"):
        namespace_str = "/" + namespace_str

    tb4_navigation = get_package_share_directory("turtlebot4_navigation")
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb4_navigation, "launch", "slam.launch.py"])
        ),
        launch_arguments=[
            ("namespace", namespace_str),
            ("sync", "true"),
            ("use_sim_time", "false"),
        ],
    )

    mapper = GroupAction([
        PushRosNamespace(namespace_str),
        SetRemap("/tf", namespace_str + "/tf"),
        SetRemap("/tf_static", namespace_str + "/tf_static"),
        Node(
            package="paper_tb4_swarm",
            executable="odom_tf_broadcaster",
            name="odom_tf_broadcaster",
            output="screen",
        ),
        Node(
            package="paper_tb4_swarm",
            executable="wall_follow_mapper",
            name="wall_follow_mapper",
            output="screen",
            parameters=[{
                "duration_sec": LaunchConfiguration("duration_sec"),
                "initial_spin_sec": LaunchConfiguration("initial_spin_sec"),
                "scan_yaw_offset_deg": LaunchConfiguration("scan_yaw_offset_deg"),
                "target_wall_distance_m": LaunchConfiguration("target_wall_distance_m"),
                "front_stop_distance_m": LaunchConfiguration("front_stop_distance_m"),
                "emergency_stop_distance_m": LaunchConfiguration("emergency_stop_distance_m"),
                "linear_speed_mps": LaunchConfiguration("linear_speed_mps"),
                "turn_speed_radps": LaunchConfiguration("turn_speed_radps"),
            }],
        ),
    ])
    return [slam, mapper]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
