import csv
import json
import math
import os
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


@dataclass
class RobotMetrics:
    last_xy: Optional[Tuple[float, float]] = None
    path_m: float = 0.0
    role: str = "unknown"


@dataclass
class TrialMetrics:
    start_sec: float
    complete: bool = False
    completion_sec: Optional[float] = None
    role_switches: int = 0
    min_separation_m: Optional[float] = None
    requirement_refs: set[str] = field(default_factory=set)
    robots: Dict[str, RobotMetrics] = field(default_factory=dict)


class MetricsLogger(Node):
    """Write paper-validation metrics for the two-robot trial."""

    def __init__(self) -> None:
        super().__init__("metrics_logger")
        self.declare_parameter("robot_namespaces", ["robot1", "robot2"])
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("role_topic", "assigned_role")
        self.declare_parameter("task_state_topic", "/swarm/task_state")
        self.declare_parameter("events_topic", "/swarm/events")
        self.declare_parameter("output_dir", "metrics")

        self.robot_namespaces = self._namespaces(
            self.get_parameter("robot_namespaces").value
        )
        self.output_dir = os.path.abspath(str(self.get_parameter("output_dir").value))
        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.summary_path = os.path.join(self.output_dir, f"summary_{stamp}.csv")
        self.timeline_path = os.path.join(self.output_dir, f"timeline_{stamp}.csv")
        self.summary_file = open(self.summary_path, "w", newline="", encoding="utf-8")
        self.timeline_file = open(self.timeline_path, "w", newline="", encoding="utf-8")
        self.summary = csv.DictWriter(
            self.summary_file,
            fieldnames=[
                "success",
                "duration_s",
                "role_switches",
                "min_separation_m",
                "robot1_path_m",
                "robot2_path_m",
                "requirement_refs",
            ],
        )
        self.timeline = csv.DictWriter(
            self.timeline_file,
            fieldnames=["stamp", "event", "robot", "role", "x", "y", "raw"],
        )
        self.summary.writeheader()
        self.timeline.writeheader()

        self.metrics = TrialMetrics(
            start_sec=self._now_sec(),
            robots={namespace: RobotMetrics() for namespace in self.robot_namespaces},
        )
        self.summary_written = False

        odom_topic = str(self.get_parameter("odom_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        role_topic = str(self.get_parameter("role_topic").value)
        for namespace in self.robot_namespaces:
            if pose_topic:
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    self._topic(namespace, pose_topic),
                    lambda msg, ns=namespace: self._pose_callback(ns, msg),
                    qos_profile_sensor_data,
                )
            else:
                self.create_subscription(
                    Odometry,
                    self._topic(namespace, odom_topic),
                    lambda msg, ns=namespace: self._odom_callback(ns, msg),
                    qos_profile_sensor_data,
                )
            self.create_subscription(
                String,
                self._topic(namespace, role_topic),
                lambda msg, ns=namespace: self._role_callback(ns, msg),
                10,
            )

        self.create_subscription(
            String,
            str(self.get_parameter("task_state_topic").value),
            self._state_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("events_topic").value),
            self._event_callback,
            10,
        )
        self.get_logger().info(f"Writing metrics to {self.output_dir}")

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
        self._update_robot_pose(
            namespace, msg.pose.pose.position.x, msg.pose.pose.position.y
        )

    def _pose_callback(self, namespace: str, msg: PoseWithCovarianceStamped) -> None:
        self._update_robot_pose(
            namespace, msg.pose.pose.position.x, msg.pose.pose.position.y
        )

    def _update_robot_pose(self, namespace: str, x: float, y: float) -> None:
        robot = self.metrics.robots[namespace]
        xy = (x, y)
        if robot.last_xy is not None:
            robot.path_m += math.hypot(xy[0] - robot.last_xy[0], xy[1] - robot.last_xy[1])
        robot.last_xy = xy
        self._update_separation()
        self._write_timeline("pose", namespace, "")

    def _role_callback(self, namespace: str, msg: String) -> None:
        robot = self.metrics.robots[namespace]
        if robot.role not in ("unknown", msg.data):
            self.metrics.role_switches += 1
        robot.role = msg.data
        self._write_timeline("role", namespace, msg.data)

    def _state_callback(self, msg: String) -> None:
        self._ingest_refs(msg.data)
        self._write_timeline("task_state", "", msg.data)

    def _event_callback(self, msg: String) -> None:
        self._ingest_refs(msg.data)
        self._write_timeline("event", "", msg.data)
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if payload.get("event") == "target_reached" and not self.metrics.complete:
            self.metrics.complete = True
            self.metrics.completion_sec = self._now_sec()
            self._write_summary()

    def _ingest_refs(self, raw: str) -> None:
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return
        for ref in payload.get("requirement_refs", []):
            self.metrics.requirement_refs.add(str(ref))

    def _update_separation(self) -> None:
        if len(self.robot_namespaces) < 2:
            return
        first = self.metrics.robots[self.robot_namespaces[0]].last_xy
        second = self.metrics.robots[self.robot_namespaces[1]].last_xy
        if first is None or second is None:
            return
        distance = math.hypot(first[0] - second[0], first[1] - second[1])
        if self.metrics.min_separation_m is None or distance < self.metrics.min_separation_m:
            self.metrics.min_separation_m = distance

    def _write_timeline(self, event: str, namespace: str, raw: str) -> None:
        robot = self.metrics.robots.get(namespace)
        xy = robot.last_xy if robot else None
        self.timeline.writerow(
            {
                "stamp": f"{self._now_sec():.3f}",
                "event": event,
                "robot": namespace,
                "role": robot.role if robot else "",
                "x": f"{xy[0]:.4f}" if xy else "",
                "y": f"{xy[1]:.4f}" if xy else "",
                "raw": raw,
            }
        )
        self.timeline_file.flush()

    def _write_summary(self) -> None:
        if self.summary_written:
            return
        duration = (self.metrics.completion_sec or self._now_sec()) - self.metrics.start_sec
        self.summary.writerow(
            {
                "success": int(self.metrics.complete),
                "duration_s": f"{duration:.3f}",
                "role_switches": self.metrics.role_switches,
                "min_separation_m": self._fmt(self.metrics.min_separation_m),
                "robot1_path_m": self._path_for(0),
                "robot2_path_m": self._path_for(1),
                "requirement_refs": ";".join(sorted(self.metrics.requirement_refs)),
            }
        )
        self.summary_file.flush()
        self.summary_written = True

    def _path_for(self, index: int) -> str:
        if len(self.robot_namespaces) <= index:
            return ""
        return f"{self.metrics.robots[self.robot_namespaces[index]].path_m:.4f}"

    def _fmt(self, value: Optional[float]) -> str:
        return "" if value is None else f"{value:.4f}"

    def destroy_node(self) -> bool:
        if not self.summary_file.closed:
            self._write_summary()
            self.summary_file.close()
            self.timeline_file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MetricsLogger()
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
