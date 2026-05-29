from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class OccupancyGridMarker(Node):
    def __init__(self) -> None:
        super().__init__("occupancy_grid_marker")
        self.declare_parameter("map_topic", "/robot1/map")
        self.declare_parameter("marker_topic", "/swarm/map_markers")
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("free_threshold", 25)
        self.declare_parameter("goal_min_clearance_m", 0.25)
        self.declare_parameter("unknown_blocks_goal", True)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), qos
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_callback,
            qos,
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        resolution = msg.info.resolution
        origin = msg.info.origin.position
        occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        free_threshold = int(self.get_parameter("free_threshold").value)
        goal_min_clearance_m = float(
            self.get_parameter("goal_min_clearance_m").value
        )
        unknown_blocks_goal = bool(self.get_parameter("unknown_blocks_goal").value)

        occupied_points: list[Point] = []
        free_points: list[Point] = []
        unsafe_goal_points: list[Point] = []
        safe_goal_points: list[Point] = []
        for y in range(msg.info.height):
            row = y * msg.info.width
            for x in range(msg.info.width):
                value = msg.data[row + x]
                if value < 0:
                    continue
                point = Point()
                point.x = origin.x + (x + 0.5) * resolution
                point.y = origin.y + (y + 0.5) * resolution
                point.z = 0.0
                if value >= occupied_threshold:
                    occupied_points.append(point)
                elif value <= free_threshold:
                    free_points.append(point)
                    if self._has_clearance(
                        msg,
                        x,
                        y,
                        goal_min_clearance_m,
                        occupied_threshold,
                        unknown_blocks_goal,
                    ):
                        safe_goal_points.append(point)
                    else:
                        unsafe_goal_points.append(point)

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        markers.markers.append(
            self._cube_list(
                msg.header.frame_id or "map",
                0,
                "free",
                free_points,
                resolution,
                ColorRGBA(r=0.62, g=0.65, b=0.68, a=0.18),
                0.006,
            )
        )
        markers.markers.append(
            self._cube_list(
                msg.header.frame_id or "map",
                1,
                "unsafe_goal",
                unsafe_goal_points,
                resolution,
                ColorRGBA(r=1.00, g=0.56, b=0.12, a=0.42),
                0.012,
            )
        )
        markers.markers.append(
            self._cube_list(
                msg.header.frame_id or "map",
                2,
                "safe_goal",
                safe_goal_points,
                resolution,
                ColorRGBA(r=0.10, g=0.88, b=0.36, a=0.58),
                0.014,
            )
        )
        markers.markers.append(
            self._cube_list(
                msg.header.frame_id or "map",
                3,
                "occupied",
                occupied_points,
                resolution,
                ColorRGBA(r=0.05, g=0.05, b=0.05, a=0.95),
                0.02,
            )
        )
        self.marker_pub.publish(markers)
        self.get_logger().info(
            f"Published map markers: {len(safe_goal_points)} safe goals, "
            f"{len(unsafe_goal_points)} unsafe free, {len(occupied_points)} occupied"
        )

    def _has_clearance(
        self,
        msg: OccupancyGrid,
        x: int,
        y: int,
        clearance_m: float,
        occupied_threshold: int,
        unknown_blocks_goal: bool,
    ) -> bool:
        resolution = float(msg.info.resolution)
        if resolution <= 0.0:
            return False

        radius_cells = int(math.ceil(clearance_m / resolution))
        origin = msg.info.origin.position
        wx = origin.x + (x + 0.5) * resolution
        wy = origin.y + (y + 0.5) * resolution

        for cy in range(y - radius_cells, y + radius_cells + 1):
            for cx in range(x - radius_cells, x + radius_cells + 1):
                if cx < 0 or cy < 0 or cx >= msg.info.width or cy >= msg.info.height:
                    if unknown_blocks_goal:
                        return False
                    continue
                check_x = origin.x + (cx + 0.5) * resolution
                check_y = origin.y + (cy + 0.5) * resolution
                if math.hypot(check_x - wx, check_y - wy) > clearance_m:
                    continue
                value = int(msg.data[cy * msg.info.width + cx])
                if value < 0 and unknown_blocks_goal:
                    return False
                if value >= occupied_threshold:
                    return False
        return True

    def _cube_list(
        self,
        frame_id: str,
        marker_id: int,
        namespace: str,
        points: list[Point],
        resolution: float,
        color: ColorRGBA,
        z_scale: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = resolution
        marker.scale.y = resolution
        marker.scale.z = z_scale
        marker.color = color
        marker.points = points
        return marker


def main() -> None:
    rclpy.init()
    node = OccupancyGridMarker()
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
