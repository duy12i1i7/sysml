import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class FrontierCandidate:
    x: float
    y: float
    yaw: float
    distance: float
    size_cells: int
    information_cells: int
    score: float


class FrontierExplorer(Node):
    """Explore an active SLAM map by sending Nav2 goals to frontier cells."""

    def __init__(self) -> None:
        super().__init__("frontier_explorer")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("selection_period_sec", 2.0)
        self.declare_parameter("free_threshold", 25)
        self.declare_parameter("occupied_threshold", 55)
        self.declare_parameter("min_frontier_size_cells", 8)
        self.declare_parameter("min_frontier_distance_m", 0.55)
        self.declare_parameter("max_frontier_distance_m", 8.0)
        self.declare_parameter("obstacle_clearance_m", 0.38)
        self.declare_parameter("information_radius_m", 0.9)
        self.declare_parameter("approach_offset_m", 0.25)
        self.declare_parameter("blacklist_radius_m", 0.55)
        self.declare_parameter("goal_timeout_sec", 75.0)
        self.declare_parameter("distance_weight", 1.0)
        self.declare_parameter("information_weight", 0.025)
        self.declare_parameter("size_weight", 0.015)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.free_threshold = int(self.get_parameter("free_threshold").value)
        self.occupied_threshold = int(
            self.get_parameter("occupied_threshold").value
        )
        self.min_frontier_size_cells = int(
            self.get_parameter("min_frontier_size_cells").value
        )
        self.min_frontier_distance_m = float(
            self.get_parameter("min_frontier_distance_m").value
        )
        self.max_frontier_distance_m = float(
            self.get_parameter("max_frontier_distance_m").value
        )
        self.obstacle_clearance_m = float(
            self.get_parameter("obstacle_clearance_m").value
        )
        self.information_radius_m = float(
            self.get_parameter("information_radius_m").value
        )
        self.approach_offset_m = float(
            self.get_parameter("approach_offset_m").value
        )
        self.blacklist_radius_m = float(
            self.get_parameter("blacklist_radius_m").value
        )
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.distance_weight = float(self.get_parameter("distance_weight").value)
        self.information_weight = float(
            self.get_parameter("information_weight").value
        )
        self.size_weight = float(self.get_parameter("size_weight").value)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_msg: Optional[OccupancyGrid] = None
        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
        )
        self.goal_pub = self.create_publisher(PoseStamped, "exploration_goal", 10)
        self.action_client = ActionClient(self, NavigateToPose, self.action_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_goal: Optional[FrontierCandidate] = None
        self.current_goal_handle = None
        self.goal_sent_time: Optional[float] = None
        self.blacklist: list[tuple[float, float]] = []
        self.no_frontier_cycles = 0

        self.create_timer(
            float(self.get_parameter("selection_period_sec").value), self._tick
        )
        self.get_logger().info(
            f"Frontier explorer waiting for {self.map_topic} and {self.action_name}"
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _tick(self) -> None:
        if self.map_msg is None:
            self.get_logger().warn(
                f"Waiting for SLAM map on {self.map_topic}",
                throttle_duration_sec=5.0,
            )
            return

        robot_pose = self._robot_pose()
        if robot_pose is None:
            return

        if self.current_goal is not None:
            if self._goal_timed_out():
                goal = self.current_goal
                self._blacklist(goal)
                self._cancel_current_goal()
                self.get_logger().warn(
                    f"Frontier goal timed out; blacklisted x={goal.x:.2f}, y={goal.y:.2f}"
                )
            return

        if not self.action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                f"Waiting for Nav2 action server {self.action_name}",
                throttle_duration_sec=5.0,
            )
            return

        candidates = self._find_frontiers(robot_pose)
        if not candidates:
            self.no_frontier_cycles += 1
            if self.no_frontier_cycles == 3:
                self.get_logger().info(
                    "No reachable frontier remains. Save the map if RViz coverage looks complete."
                )
            else:
                self.get_logger().info(
                    "No reachable frontier candidate found",
                    throttle_duration_sec=10.0,
                )
            return

        self.no_frontier_cycles = 0
        best = max(candidates, key=lambda candidate: candidate.score)
        self._send_goal(best)

    def _robot_pose(self) -> Optional[tuple[float, float]]:
        frames = [self.base_frame]
        namespace = self.get_namespace().strip("/")
        if namespace and "/" not in self.base_frame:
            frames.append(f"{namespace}/{self.base_frame}")

        for frame in frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    frame,
                    Time(),
                )
                t = transform.transform.translation
                return (float(t.x), float(t.y))
            except TransformException:
                continue

        self.get_logger().warn(
            f"Waiting for TF {self.map_frame} -> {frames}",
            throttle_duration_sec=5.0,
        )
        return None

    def _goal_timed_out(self) -> bool:
        if self.goal_sent_time is None:
            return False
        return self.get_clock().now().nanoseconds * 1.0e-9 - self.goal_sent_time > (
            self.goal_timeout_sec
        )

    def _send_goal(self, candidate: FrontierCandidate) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._pose(candidate)
        self.goal_pub.publish(goal_msg.pose)
        self.current_goal = candidate
        self.goal_sent_time = self.get_clock().now().nanoseconds * 1.0e-9
        send_future = self.action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(
            "Exploring frontier "
            f"x={candidate.x:.2f}, y={candidate.y:.2f}, "
            f"dist={candidate.distance:.2f}m, info={candidate.information_cells}, "
            f"size={candidate.size_cells}, score={candidate.score:.2f}"
        )

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            # Rejection can happen during Nav2 lifecycle activation. Do not
            # blacklist the frontier until Nav2 accepts it and returns failure.
            self.current_goal = None
            self.current_goal_handle = None
            self.goal_sent_time = None
            self.get_logger().warn("Nav2 rejected frontier goal; will retry")
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        result = future.result()
        goal = self.current_goal
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if goal is not None:
                self.get_logger().info(
                    f"Reached frontier x={goal.x:.2f}, y={goal.y:.2f}"
                )
        else:
            if goal is not None:
                self._blacklist(goal)
                self.get_logger().warn(
                    "Frontier goal failed; blacklisted "
                    f"x={goal.x:.2f}, y={goal.y:.2f}, status={result.status}"
                )

        self.current_goal = None
        self.current_goal_handle = None
        self.goal_sent_time = None

    def _cancel_current_goal(self) -> None:
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
        self.current_goal = None
        self.current_goal_handle = None
        self.goal_sent_time = None

    def _find_frontiers(
        self, robot_pose: tuple[float, float]
    ) -> list[FrontierCandidate]:
        assert self.map_msg is not None
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        resolution = self.map_msg.info.resolution
        data = self.map_msg.data
        if width == 0 or height == 0 or resolution <= 0.0:
            return []

        clearance_cells = max(1, math.ceil(self.obstacle_clearance_m / resolution))
        information_cells = max(1, math.ceil(self.information_radius_m / resolution))
        frontier = bytearray(width * height)
        for y in range(height):
            for x in range(width):
                idx = self._index(width, x, y)
                if not self._is_free(data[idx]):
                    continue
                if not self._has_unknown_neighbor(data, width, height, x, y):
                    continue
                if not self._has_obstacle_clearance(
                    data, width, height, x, y, clearance_cells
                ):
                    continue
                frontier[idx] = 1

        visited = bytearray(width * height)
        candidates: list[FrontierCandidate] = []
        for y in range(height):
            for x in range(width):
                idx = self._index(width, x, y)
                if not frontier[idx] or visited[idx]:
                    continue
                component = self._collect_component(frontier, visited, width, height, x, y)
                if len(component) < self.min_frontier_size_cells:
                    continue
                candidate = self._component_candidate(
                    component,
                    data,
                    width,
                    height,
                    resolution,
                    information_cells,
                    robot_pose,
                )
                if candidate is not None:
                    candidates.append(candidate)
        return candidates

    def _collect_component(
        self,
        frontier: bytearray,
        visited: bytearray,
        width: int,
        height: int,
        start_x: int,
        start_y: int,
    ) -> list[tuple[int, int]]:
        queue = deque([(start_x, start_y)])
        visited[self._index(width, start_x, start_y)] = 1
        cells: list[tuple[int, int]] = []
        while queue:
            x, y = queue.popleft()
            cells.append((x, y))
            for nx, ny in self._neighbors8(width, height, x, y):
                idx = self._index(width, nx, ny)
                if frontier[idx] and not visited[idx]:
                    visited[idx] = 1
                    queue.append((nx, ny))
        return cells

    def _component_candidate(
        self,
        component: list[tuple[int, int]],
        data,
        width: int,
        height: int,
        resolution: float,
        information_radius_cells: int,
        robot_pose: tuple[float, float],
    ) -> Optional[FrontierCandidate]:
        best: Optional[FrontierCandidate] = None
        for x, y in component:
            frontier_x, frontier_y = self._map_to_world(x, y)
            approach = self._approach_goal(
                data,
                width,
                height,
                resolution,
                x,
                y,
                frontier_x,
                frontier_y,
                robot_pose,
            )
            if approach is None:
                continue
            goal_x, goal_y, goal_map_x, goal_map_y = approach
            distance = math.hypot(goal_x - robot_pose[0], goal_y - robot_pose[1])
            if distance < self.min_frontier_distance_m:
                continue
            if distance > self.max_frontier_distance_m:
                continue
            if self._is_blacklisted(goal_x, goal_y):
                continue

            info = self._count_unknown(
                data,
                width,
                height,
                goal_map_x,
                goal_map_y,
                information_radius_cells,
            )
            yaw = self._yaw_toward_unknown(
                data,
                width,
                height,
                resolution,
                x,
                y,
                goal_x,
                goal_y,
            )
            score = (
                self.information_weight * info
                + self.size_weight * len(component)
                - self.distance_weight * distance
            )
            candidate = FrontierCandidate(
                x=goal_x,
                y=goal_y,
                yaw=yaw,
                distance=distance,
                size_cells=len(component),
                information_cells=info,
                score=score,
            )
            if best is None or candidate.score > best.score:
                best = candidate
        return best

    def _approach_goal(
        self,
        data,
        width: int,
        height: int,
        resolution: float,
        frontier_map_x: int,
        frontier_map_y: int,
        frontier_x: float,
        frontier_y: float,
        robot_pose: tuple[float, float],
    ) -> Optional[tuple[float, float, int, int]]:
        dx = robot_pose[0] - frontier_x
        dy = robot_pose[1] - frontier_y
        distance = math.hypot(dx, dy)
        if distance < 1.0e-6:
            return (frontier_x, frontier_y, frontier_map_x, frontier_map_y)

        approach_x = frontier_x + self.approach_offset_m * dx / distance
        approach_y = frontier_y + self.approach_offset_m * dy / distance
        approach_cell = self._world_to_map(approach_x, approach_y)
        if approach_cell is None:
            approach_cell = (frontier_map_x, frontier_map_y)

        clearance_cells = max(1, math.ceil(self.obstacle_clearance_m / resolution))
        map_margin_cells = max(clearance_cells, math.ceil(0.22 / resolution))
        search_radius = max(1, math.ceil(self.approach_offset_m / resolution))
        best = None
        best_distance = float("inf")
        center_x, center_y = approach_cell
        for y in range(
            max(0, center_y - search_radius),
            min(height, center_y + search_radius + 1),
        ):
            for x in range(
                max(0, center_x - search_radius),
                min(width, center_x + search_radius + 1),
            ):
                value = data[self._index(width, x, y)]
                if not self._is_free(value):
                    continue
                if not self._has_map_margin(width, height, x, y, map_margin_cells):
                    continue
                if not self._has_obstacle_clearance(
                    data, width, height, x, y, clearance_cells
                ):
                    continue
                world_x, world_y = self._map_to_world(x, y)
                candidate_distance = math.hypot(world_x - approach_x, world_y - approach_y)
                if candidate_distance < best_distance:
                    best = (world_x, world_y, x, y)
                    best_distance = candidate_distance
        return best

    def _map_to_world(self, x: int, y: int) -> tuple[float, float]:
        assert self.map_msg is not None
        info = self.map_msg.info
        origin = info.origin.position
        return (
            origin.x + (x + 0.5) * info.resolution,
            origin.y + (y + 0.5) * info.resolution,
        )

    def _world_to_map(self, x: float, y: float) -> Optional[tuple[int, int]]:
        assert self.map_msg is not None
        info = self.map_msg.info
        mx = math.floor((x - info.origin.position.x) / info.resolution)
        my = math.floor((y - info.origin.position.y) / info.resolution)
        if 0 <= mx < info.width and 0 <= my < info.height:
            return (int(mx), int(my))
        return None

    def _pose(self, candidate: FrontierCandidate) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        # Use latest TF. On physical TurtleBot4 runs, SLAM's map->odom update can
        # lag goal creation enough for Nav2 to reject a precisely timestamped goal.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.pose.position.x = candidate.x
        msg.pose.position.y = candidate.y
        msg.pose.orientation.z = math.sin(candidate.yaw * 0.5)
        msg.pose.orientation.w = math.cos(candidate.yaw * 0.5)
        return msg

    def _yaw_toward_unknown(
        self,
        data,
        width: int,
        height: int,
        resolution: float,
        x: int,
        y: int,
        world_x: float,
        world_y: float,
    ) -> float:
        unknown_x = 0.0
        unknown_y = 0.0
        count = 0
        radius = max(2, math.ceil(0.55 / resolution))
        for ny in range(max(0, y - radius), min(height, y + radius + 1)):
            for nx in range(max(0, x - radius), min(width, x + radius + 1)):
                if self._is_unknown(data[self._index(width, nx, ny)]):
                    wx, wy = self._map_to_world(nx, ny)
                    unknown_x += wx
                    unknown_y += wy
                    count += 1
        if count == 0:
            return 0.0
        return math.atan2(unknown_y / count - world_y, unknown_x / count - world_x)

    def _count_unknown(
        self,
        data,
        width: int,
        height: int,
        x: int,
        y: int,
        radius: int,
    ) -> int:
        total = 0
        for ny in range(max(0, y - radius), min(height, y + radius + 1)):
            for nx in range(max(0, x - radius), min(width, x + radius + 1)):
                if self._is_unknown(data[self._index(width, nx, ny)]):
                    total += 1
        return total

    def _has_obstacle_clearance(
        self,
        data,
        width: int,
        height: int,
        x: int,
        y: int,
        radius: int,
    ) -> bool:
        for ny in range(max(0, y - radius), min(height, y + radius + 1)):
            for nx in range(max(0, x - radius), min(width, x + radius + 1)):
                if math.hypot(nx - x, ny - y) > radius:
                    continue
                if self._is_occupied(data[self._index(width, nx, ny)]):
                    return False
        return True

    def _has_map_margin(
        self, width: int, height: int, x: int, y: int, radius: int
    ) -> bool:
        return radius <= x < width - radius and radius <= y < height - radius

    def _has_unknown_neighbor(self, data, width: int, height: int, x: int, y: int) -> bool:
        for nx, ny in self._neighbors8(width, height, x, y):
            if self._is_unknown(data[self._index(width, nx, ny)]):
                return True
        return False

    def _neighbors8(self, width: int, height: int, x: int, y: int):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    yield nx, ny

    def _index(self, width: int, x: int, y: int) -> int:
        return y * width + x

    def _is_free(self, value: int) -> bool:
        return 0 <= int(value) <= self.free_threshold

    def _is_unknown(self, value: int) -> bool:
        return int(value) < 0

    def _is_occupied(self, value: int) -> bool:
        return int(value) >= self.occupied_threshold

    def _is_blacklisted(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - bx, y - by) < self.blacklist_radius_m
            for bx, by in self.blacklist
        )

    def _blacklist(self, goal: FrontierCandidate) -> None:
        if not self._is_blacklisted(goal.x, goal.y):
            self.blacklist.append((goal.x, goal.y))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.current_goal_handle is not None:
            try:
                node.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
