import json
import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from paper_tb4_swarm.mbse_model import Phase, Role, metadata


@dataclass
class RobotState:
    x: Optional[float] = None
    y: Optional[float] = None
    last_odom_sec: float = 0.0
    role: str = Role.IDLE.value
    last_goal: Optional[Tuple[float, float]] = None


class Coordinator(Node):
    """Assign leader/follower goals for two TurtleBot4 robots."""

    def __init__(self) -> None:
        super().__init__("paper_swarm_coordinator")
        self.declare_parameter("robot_namespaces", ["robot1", "robot2"])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("role_topic", "assigned_role")
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("formation_mode", "leader_follower")
        self.declare_parameter("leader_namespace", "robot1")
        self.declare_parameter("follower_distance_m", 0.8)
        self.declare_parameter("follower_lateral_offset_m", 0.0)
        self.declare_parameter("assignment_period_sec", 0.5)
        self.declare_parameter("odom_timeout_sec", 2.0)
        self.declare_parameter("target_reached_tolerance_m", 0.25)

        self.robot_namespaces = self._namespaces(
            self.get_parameter("robot_namespaces").value
        )
        if len(self.robot_namespaces) != 2:
            raise ValueError(
                "paper_swarm_coordinator is scoped to exactly two TurtleBot4 namespaces"
            )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.role_topic = str(self.get_parameter("role_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.formation_mode = str(self.get_parameter("formation_mode").value)
        self.leader_namespace = str(self.get_parameter("leader_namespace").value).strip(
            "/"
        )
        if self.leader_namespace not in self.robot_namespaces:
            raise ValueError(
                f"leader_namespace must be one of {self.robot_namespaces}"
            )
        self.follower_distance_m = float(
            self.get_parameter("follower_distance_m").value
        )
        self.follower_lateral_offset_m = float(
            self.get_parameter("follower_lateral_offset_m").value
        )
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self.target_reached_tolerance_m = float(
            self.get_parameter("target_reached_tolerance_m").value
        )

        self.robots: Dict[str, RobotState] = {
            namespace: RobotState() for namespace in self.robot_namespaces
        }
        self.role_publishers = {}
        self.goal_publishers = {}
        for namespace in self.robot_namespaces:
            self.create_subscription(
                Odometry,
                self._topic(namespace, self.odom_topic),
                lambda msg, ns=namespace: self._odom_callback(ns, msg),
                qos_profile_sensor_data,
            )
            if self.pose_topic:
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    self._topic(namespace, self.pose_topic),
                    lambda msg, ns=namespace: self._pose_callback(ns, msg),
                    qos_profile_sensor_data,
                )
            self.role_publishers[namespace] = self.create_publisher(
                String, self._topic(namespace, self.role_topic), 10
            )
            self.goal_publishers[namespace] = self.create_publisher(
                PoseStamped, self._topic(namespace, self.goal_topic), 10
            )

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("target_topic").value),
            self._target_callback,
            10,
        )
        self.task_state_pub = self.create_publisher(String, "/swarm/task_state", 10)
        self.events_pub = self.create_publisher(String, "/swarm/events", 10)

        self.target: Optional[Tuple[float, float]] = None
        self.last_leader_direction: Tuple[float, float] = (1.0, 0.0)
        self.task_complete = False
        self.create_timer(
            float(self.get_parameter("assignment_period_sec").value), self._tick
        )
        self.get_logger().info(
            f"Coordinator ready for {', '.join(self.robot_namespaces)}"
        )

    def _namespaces(self, value) -> list[str]:
        if isinstance(value, str):
            items = value.split(",")
        else:
            items = value
        return [str(item).strip().strip("/") for item in items if str(item).strip()]

    def _topic(self, namespace: str, topic: str) -> str:
        return f"/{namespace}/{topic.lstrip('/')}"

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _odom_callback(self, namespace: str, msg: Odometry) -> None:
        robot = self.robots[namespace]
        robot.last_odom_sec = self._now_sec()
        if not self.pose_topic:
            robot.x = msg.pose.pose.position.x
            robot.y = msg.pose.pose.position.y

    def _pose_callback(self, namespace: str, msg: PoseWithCovarianceStamped) -> None:
        robot = self.robots[namespace]
        robot.x = msg.pose.pose.position.x
        robot.y = msg.pose.pose.position.y

    def _target_callback(self, msg: PoseStamped) -> None:
        self.target = (msg.pose.position.x, msg.pose.position.y)
        self.task_complete = False
        self._event("target_updated", Phase.ESTIMATE_STATE, {"target": self.target})

    def _tick(self) -> None:
        if self.target is None:
            self._state("waiting_for_target", Phase.WAITING_FOR_TARGET)
            return

        available = self._available_robots()
        if len(available) < len(self.robot_namespaces):
            for namespace, robot in self.robots.items():
                if namespace not in available and robot.role != Role.IDLE.value:
                    self._set_role(namespace, Role.IDLE)
            self._state("waiting_for_fresh_odometry", Phase.ESTIMATE_STATE)
            return

        if self.formation_mode != "leader_follower":
            raise ValueError("formation_mode must be 'leader_follower'")
        self._tick_leader_follower()

    def _tick_leader_follower(self) -> None:
        leader = self.leader_namespace
        follower = next(ns for ns in self.robot_namespaces if ns != leader)
        self._set_role(leader, Role.LEADER)
        self._set_role(follower, Role.FOLLOWER)

        self._publish_goal(leader, self.target)

        leader_state = self.robots[leader]
        follower_goal = self._follower_goal(
            (leader_state.x, leader_state.y), self.target
        )
        self._publish_goal(follower, follower_goal)

        distance = self._distance((leader_state.x, leader_state.y), self.target)
        if distance <= self.target_reached_tolerance_m and not self.task_complete:
            self.task_complete = True
            self._event("target_reached", Phase.TARGET_REACHED, {"leader": leader})

        self._state("assigned_leader_follower", Phase.NAVIGATE)

    def _follower_goal(
        self, leader_xy: Tuple[float, float], target_xy: Tuple[float, float]
    ) -> Tuple[float, float]:
        dx = target_xy[0] - leader_xy[0]
        dy = target_xy[1] - leader_xy[1]
        norm = math.hypot(dx, dy)
        if norm > 0.05:
            ux = dx / norm
            uy = dy / norm
            self.last_leader_direction = (ux, uy)
        else:
            ux, uy = self.last_leader_direction

        px = -uy
        py = ux
        return (
            leader_xy[0]
            - ux * self.follower_distance_m
            + px * self.follower_lateral_offset_m,
            leader_xy[1]
            - uy * self.follower_distance_m
            + py * self.follower_lateral_offset_m,
        )

    def _available_robots(self) -> list[str]:
        now = self._now_sec()
        return [
            namespace
            for namespace, robot in self.robots.items()
            if robot.x is not None
            and robot.y is not None
            and now - robot.last_odom_sec <= self.odom_timeout_sec
        ]

    def _set_role(self, namespace: str, role: Role) -> None:
        self.robots[namespace].role = role.value
        msg = String()
        msg.data = role.value
        self.role_publishers[namespace].publish(msg)

    def _publish_goal(self, namespace: str, xy: Tuple[float, float]) -> None:
        if self.robots[namespace].last_goal == xy:
            return
        self.robots[namespace].last_goal = xy
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(xy[0])
        msg.pose.position.y = float(xy[1])
        msg.pose.orientation.w = 1.0
        self.goal_publishers[namespace].publish(msg)

    def _state(self, reason: str, phase: Phase) -> None:
        payload = {
            "stamp": self._now_sec(),
            "reason": reason,
            "target": self._point(self.target),
            "leader": self.leader_namespace,
            "follower": next(
                ns for ns in self.robot_namespaces if ns != self.leader_namespace
            ),
            "task_complete": self.task_complete,
            **metadata(phase),
            "robots": {
                namespace: {
                    "x": robot.x,
                    "y": robot.y,
                    "role": robot.role,
                    "last_goal": self._point(robot.last_goal),
                }
                for namespace, robot in self.robots.items()
            },
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.task_state_pub.publish(msg)

    def _event(self, event: str, phase: Phase, extra: dict) -> None:
        payload = {"stamp": self._now_sec(), "event": event, **metadata(phase)}
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.events_pub.publish(msg)

    def _point(self, point: Optional[Tuple[float, float]]) -> Optional[dict]:
        if point is None:
            return None
        return {"x": float(point[0]), "y": float(point[1])}

    def _distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Coordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
