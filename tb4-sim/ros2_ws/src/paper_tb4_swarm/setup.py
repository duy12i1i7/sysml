from glob import glob
from setuptools import find_packages, setup
import os


package_name = "paper_tb4_swarm"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob(os.path.join("launch", "*.launch.py"))),
        (f"share/{package_name}/config", glob(os.path.join("config", "*.yaml"))),
        (f"share/{package_name}/maps", glob(os.path.join("maps", "*"))),
        (f"share/{package_name}/worlds", glob(os.path.join("worlds", "*.sdf"))),
        (f"share/{package_name}/docs", glob(os.path.join("docs", "*.md"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Avis",
    maintainer_email="avis@example.com",
    description="Two TurtleBot4 sim-to-real swarm experiment aligned with the MBSE paper mindset.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "coordinator = paper_tb4_swarm.coordinator:main",
            "target_publisher = paper_tb4_swarm.target_publisher:main",
            "rviz_target_bridge = paper_tb4_swarm.rviz_target_bridge:main",
            "nav2_goal_dispatcher = paper_tb4_swarm.nav2_goal_dispatcher:main",
            "real_graph_check = paper_tb4_swarm.real_graph_check:main",
            "frontier_explorer = paper_tb4_swarm.frontier_explorer:main",
            "clearance_explorer = paper_tb4_swarm.clearance_explorer:main",
            "cmd_vel_relay = paper_tb4_swarm.cmd_vel_relay:main",
            "odom_tf_broadcaster = paper_tb4_swarm.odom_tf_broadcaster:main",
            "wall_follow_mapper = paper_tb4_swarm.wall_follow_mapper:main",
            "simple_goal_follower = paper_tb4_swarm.simple_goal_follower:main",
            "metrics_logger = paper_tb4_swarm.metrics_logger:main",
            "odom_offsetter = paper_tb4_swarm.odom_offsetter:main",
            "occupancy_grid_marker = paper_tb4_swarm.occupancy_grid_marker:main",
            "static_map_publisher = paper_tb4_swarm.static_map_publisher:main",
        ],
    },
)
