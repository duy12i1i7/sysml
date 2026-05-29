from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap


ARGUMENTS = [
    DeclareLaunchArgument("namespace", default_value="/robot1"),
    DeclareLaunchArgument("scan_yaw_offset_deg", default_value="0.0"),
    DeclareLaunchArgument("duration_sec", default_value="600.0"),
    DeclareLaunchArgument("initial_spin_sec", default_value="0.0"),
    DeclareLaunchArgument("initial_scan_turn_rad", default_value="6.4"),
    DeclareLaunchArgument("rescan_turn_rad", default_value="1.05"),
    DeclareLaunchArgument("drive_step_m", default_value="0.18"),
    DeclareLaunchArgument("max_drive_step_sec", default_value="18.0"),
    DeclareLaunchArgument("front_clearance_m", default_value="0.42"),
    DeclareLaunchArgument("side_clearance_m", default_value="0.18"),
    DeclareLaunchArgument("emergency_stop_distance_m", default_value="0.20"),
    DeclareLaunchArgument("linear_speed_mps", default_value="0.08"),
    DeclareLaunchArgument("turn_speed_radps", default_value="0.24"),
    DeclareLaunchArgument("turn_heading_tolerance_deg", default_value="18.0"),
    DeclareLaunchArgument("stuck_window_sec", default_value="12.0"),
    DeclareLaunchArgument("stuck_distance_m", default_value="0.012"),
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

    explorer = GroupAction([
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
            executable="clearance_explorer",
            name="clearance_explorer",
            output="screen",
            parameters=[{
                "duration_sec": LaunchConfiguration("duration_sec"),
                "initial_spin_sec": LaunchConfiguration("initial_spin_sec"),
                "initial_scan_turn_rad": LaunchConfiguration("initial_scan_turn_rad"),
                "rescan_turn_rad": LaunchConfiguration("rescan_turn_rad"),
                "drive_step_m": LaunchConfiguration("drive_step_m"),
                "max_drive_step_sec": LaunchConfiguration("max_drive_step_sec"),
                "scan_yaw_offset_deg": LaunchConfiguration("scan_yaw_offset_deg"),
                "front_clearance_m": LaunchConfiguration("front_clearance_m"),
                "side_clearance_m": LaunchConfiguration("side_clearance_m"),
                "emergency_stop_distance_m": LaunchConfiguration("emergency_stop_distance_m"),
                "linear_speed_mps": LaunchConfiguration("linear_speed_mps"),
                "turn_speed_radps": LaunchConfiguration("turn_speed_radps"),
                "turn_heading_tolerance_deg": LaunchConfiguration("turn_heading_tolerance_deg"),
                "stuck_window_sec": LaunchConfiguration("stuck_window_sec"),
                "stuck_distance_m": LaunchConfiguration("stuck_distance_m"),
            }],
        ),
    ])
    return [slam, explorer]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
