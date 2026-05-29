from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap


ARGUMENTS = [
    DeclareLaunchArgument("namespace", default_value="/robot1"),
    DeclareLaunchArgument("start_rviz", default_value="true", choices=["true", "false"]),
    DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
]


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace")
    namespace_str = namespace.perform(context)
    if namespace_str and not namespace_str.startswith("/"):
        namespace_str = "/" + namespace_str

    use_sim_time = LaunchConfiguration("use_sim_time")
    tb4_navigation = get_package_share_directory("turtlebot4_navigation")
    tb4_viz = get_package_share_directory("turtlebot4_viz")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb4_navigation, "launch", "slam.launch.py"])
        ),
        launch_arguments=[
            ("namespace", namespace_str),
            ("sync", "true"),
            ("use_sim_time", use_sim_time),
        ],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb4_viz, "launch", "view_navigation.launch.py"])
        ),
        launch_arguments=[
            ("namespace", namespace_str),
            ("use_sim_time", use_sim_time),
        ],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
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

    return [slam, tf_group, rviz]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
