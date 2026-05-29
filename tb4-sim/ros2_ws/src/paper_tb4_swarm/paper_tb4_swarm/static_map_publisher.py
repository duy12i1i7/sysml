from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from paper_tb4_swarm.map_utils import OccupancyMap


class StaticMapPublisher(Node):
    """Publish the saved lab occupancy map for stable simulation and RViz."""

    def __init__(self) -> None:
        super().__init__("static_map_publisher")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("publish_period_sec", 1.0)

        map_yaml = str(self.get_parameter("map_yaml").value)
        if not map_yaml:
            raise ValueError("static_map_publisher requires map_yaml")
        self.map = OccupancyMap.from_yaml(map_yaml)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("map_topic").value), qos
        )
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value), self._publish
        )
        self._publish()
        self.get_logger().info(
            f"Publishing static map {map_yaml} on "
            f"{self.get_parameter('map_topic').value}"
        )

    def _publish(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info = MapMetaData()
        msg.info.map_load_time = self.get_clock().now().to_msg()
        msg.info.resolution = self.map.resolution
        msg.info.width = self.map.width
        msg.info.height = self.map.height
        msg.info.origin = Pose()
        msg.info.origin.position.x = self.map.origin_x
        msg.info.origin.position.y = self.map.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map.data
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
