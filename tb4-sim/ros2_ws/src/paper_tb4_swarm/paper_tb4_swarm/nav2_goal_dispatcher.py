import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


@dataclass
class GoalState:
    pending: Optional[PoseStamped] = None
    last_sent_xy: Optional[Tuple[float, float]] = None
    map: Optional[OccupancyGrid] = None


class Nav2GoalDispatcher(Node):
    """Send coordinator-assigned per-robot goals to namespaced Nav2 actions."""

    def __init__(self) -> None:
        super().__init__("nav2_goal_dispatcher")
        self.declare_parameter("robot_namespaces", ["robot1", "robot2"])
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("dispatch_period_sec", 0.5)
        self.declare_parameter("goal_change_tolerance_m", 0.05)
        self.declare_parameter("goal_min_clearance_m", 0.25)
        self.declare_parameter("goal_occupied_threshold", 50)
        self.declare_parameter("reject_unknown_goals", True)

        self.robot_namespaces = self._namespaces(
            self.get_parameter("robot_namespaces").value
        )
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.goal_change_tolerance_m = float(
            self.get_parameter("goal_change_tolerance_m").value
        )
        self.goal_min_clearance_m = float(
            self.get_parameter("goal_min_clearance_m").value
        )
        self.goal_occupied_threshold = int(
            self.get_parameter("goal_occupied_threshold").value
        )
        self.reject_unknown_goals = bool(
            self.get_parameter("reject_unknown_goals").value
        )

        self.action_clients: Dict[str, ActionClient] = {}
        self.states: Dict[str, GoalState] = {}
        for namespace in self.robot_namespaces:
            self.action_clients[namespace] = ActionClient(
                self,
                NavigateToPose,
                self._topic(namespace, self.action_name),
            )
            self.states[namespace] = GoalState()
            self.create_subscription(
                PoseStamped,
                self._topic(namespace, self.goal_topic),
                lambda msg, ns=namespace: self._goal_callback(ns, msg),
                10,
            )
            self.create_subscription(
                OccupancyGrid,
                self._topic(namespace, self.map_topic),
                lambda msg, ns=namespace: self._map_callback(ns, msg),
                10,
            )

        self.create_timer(
            float(self.get_parameter("dispatch_period_sec").value), self._tick
        )
        self.get_logger().info(
            "Nav2 dispatcher ready for "
            + ", ".join(f"/{ns}/{self.action_name}" for ns in self.robot_namespaces)
        )

    def _namespaces(self, value) -> list[str]:
        if isinstance(value, str):
            items = value.split(",")
        else:
            items = value
        return [str(item).strip().strip("/") for item in items if str(item).strip()]

    def _topic(self, namespace: str, topic: str) -> str:
        namespace = namespace.strip().strip("/")
        topic = topic.strip().strip("/")
        return f"/{namespace}/{topic}"

    def _goal_callback(self, namespace: str, msg: PoseStamped) -> None:
        state = self.states[namespace]
        xy = (msg.pose.position.x, msg.pose.position.y)
        valid, reason = self._goal_is_clear(state.map, xy)
        if not valid:
            self.get_logger().warn(
                f"Rejecting Nav2 goal for {namespace}: x={xy[0]:.2f}, y={xy[1]:.2f} "
                f"is not safely reachable ({reason})"
            )
            state.pending = None
            return
        if (
            state.pending is None
            and state.last_sent_xy is not None
            and self._distance(xy, state.last_sent_xy) < self.goal_change_tolerance_m
        ):
            return
        target = PoseStamped()
        target.header = msg.header
        target.header.stamp = self.get_clock().now().to_msg()
        if not target.header.frame_id:
            target.header.frame_id = self.map_frame
        target.pose = msg.pose
        state.pending = target

    def _map_callback(self, namespace: str, msg: OccupancyGrid) -> None:
        self.states[namespace].map = msg

    def _tick(self) -> None:
        for namespace, state in self.states.items():
            if state.pending is None:
                continue
            client = self.action_clients[namespace]
            if not client.wait_for_server(timeout_sec=0.0):
                self.get_logger().warn(
                    f"Waiting for Nav2 action server /{namespace}/{self.action_name}",
                    throttle_duration_sec=5.0,
                )
                continue

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = state.pending
            send_future = client.send_goal_async(goal_msg)
            send_future.add_done_callback(
                lambda future, ns=namespace: self._goal_response_callback(ns, future)
            )
            state.last_sent_xy = (
                state.pending.pose.position.x,
                state.pending.pose.position.y,
            )
            state.pending = None
            self.get_logger().info(
                f"Sent Nav2 goal to {namespace}: "
                f"x={state.last_sent_xy[0]:.2f}, y={state.last_sent_xy[1]:.2f}"
            )

    def _goal_response_callback(self, namespace: str, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Nav2 rejected goal for {namespace}")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, ns=namespace: self._result_callback(ns, result)
        )

    def _result_callback(self, namespace: str, future) -> None:
        result = future.result().result
        if result.error_code == NavigateToPose.Result.NONE:
            self.get_logger().info(f"Nav2 completed goal for {namespace}")
        else:
            self.get_logger().warn(
                f"Nav2 result for {namespace}: "
                f"error_code={result.error_code}, error_msg={result.error_msg}"
            )

    def _distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))

    def _goal_is_clear(
        self, map_msg: Optional[OccupancyGrid], xy: Tuple[float, float]
    ) -> Tuple[bool, str]:
        if map_msg is None:
            return True, "map not received yet"

        info = map_msg.info
        resolution = float(info.resolution)
        if resolution <= 0.0:
            return False, "invalid map resolution"

        mx = int(math.floor((float(xy[0]) - info.origin.position.x) / resolution))
        my = int(math.floor((float(xy[1]) - info.origin.position.y) / resolution))
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return False, "outside map"

        radius_cells = max(0, int(math.ceil(self.goal_min_clearance_m / resolution)))
        for cy in range(max(0, my - radius_cells), min(info.height, my + radius_cells + 1)):
            for cx in range(max(0, mx - radius_cells), min(info.width, mx + radius_cells + 1)):
                wx = info.origin.position.x + (cx + 0.5) * resolution
                wy = info.origin.position.y + (cy + 0.5) * resolution
                if self._distance((wx, wy), xy) > self.goal_min_clearance_m:
                    continue
                value = int(map_msg.data[cy * info.width + cx])
                if value < 0 and self.reject_unknown_goals:
                    return False, f"unknown cell within {self.goal_min_clearance_m:.2f} m"
                if value >= self.goal_occupied_threshold:
                    return False, f"obstacle within {self.goal_min_clearance_m:.2f} m"

        return True, "clear"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2GoalDispatcher()
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
