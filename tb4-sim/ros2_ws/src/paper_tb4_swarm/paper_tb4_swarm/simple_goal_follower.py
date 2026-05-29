import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
    TwistStamped,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from paper_tb4_swarm.map_utils import OccupancyMap


@dataclass
class FollowerState:
    odom: Optional[Tuple[float, float, float]] = None
    goal: Optional[Tuple[float, float]] = None
    planned_goal: Optional[Tuple[float, float]] = None
    planned_from: Optional[Tuple[int, int]] = None
    waypoints: list[Tuple[float, float]] = None
    last_plan_sec: float = 0.0
    last_warning_sec: float = 0.0

    def __post_init__(self) -> None:
        if self.waypoints is None:
            self.waypoints = []


class SimpleGoalFollower(Node):
    """Small simulation-only controller for goal_pose topics."""

    def __init__(self) -> None:
        super().__init__("simple_goal_follower")
        self.declare_parameter("robot_namespaces", ["robot1", "robot2"])
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("cmd_vel_topics", "")
        self.declare_parameter("cmd_vel_unstamped_topics", [""])
        self.declare_parameter("cmd_vel_stamped_topics", [""])
        self.declare_parameter("cmd_vel_stamped", False)
        self.declare_parameter("control_period_sec", 0.1)
        self.declare_parameter("goal_tolerance_m", 0.12)
        self.declare_parameter("heading_tolerance_rad", 0.3)
        self.declare_parameter("linear_gain", 0.8)
        self.declare_parameter("angular_gain", 2.0)
        self.declare_parameter("max_linear_velocity", 0.25)
        self.declare_parameter("max_angular_velocity", 1.2)
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_clearance_m", 0.16)
        self.declare_parameter("map_unknown_is_obstacle", True)
        self.declare_parameter("map_waypoint_tolerance_m", 0.08)
        self.declare_parameter("map_replan_period_sec", 1.0)
        self.declare_parameter("min_robot_separation_m", 0.32)
        self.declare_parameter("slow_robot_separation_m", 0.45)
        self.declare_parameter("separation_linear_velocity", 0.08)

        self.robot_namespaces = self._namespaces(
            self.get_parameter("robot_namespaces").value
        )
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        legacy_cmd_vel_topics = self._topics(
            self.get_parameter("cmd_vel_topics").value,
            str(self.get_parameter("cmd_vel_topic").value),
        )
        self.cmd_vel_stamped = bool(self.get_parameter("cmd_vel_stamped").value)
        self.cmd_vel_unstamped_topics = self._topics(
            self.get_parameter("cmd_vel_unstamped_topics").value,
            "",
            allow_empty=True,
        )
        self.cmd_vel_stamped_topics = self._topics(
            self.get_parameter("cmd_vel_stamped_topics").value,
            "",
            allow_empty=True,
        )
        if not self.cmd_vel_unstamped_topics and not self.cmd_vel_stamped_topics:
            if self.cmd_vel_stamped:
                self.cmd_vel_stamped_topics = legacy_cmd_vel_topics
            else:
                self.cmd_vel_unstamped_topics = legacy_cmd_vel_topics
        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)
        self.heading_tolerance_rad = float(
            self.get_parameter("heading_tolerance_rad").value
        )
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_linear_velocity = float(
            self.get_parameter("max_linear_velocity").value
        )
        self.max_angular_velocity = float(
            self.get_parameter("max_angular_velocity").value
        )
        self.map_clearance_m = float(self.get_parameter("map_clearance_m").value)
        self.map_unknown_is_obstacle = bool(
            self.get_parameter("map_unknown_is_obstacle").value
        )
        self.map_waypoint_tolerance_m = float(
            self.get_parameter("map_waypoint_tolerance_m").value
        )
        self.map_replan_period_sec = float(
            self.get_parameter("map_replan_period_sec").value
        )
        self.min_robot_separation_m = float(
            self.get_parameter("min_robot_separation_m").value
        )
        self.slow_robot_separation_m = float(
            self.get_parameter("slow_robot_separation_m").value
        )
        self.separation_linear_velocity = float(
            self.get_parameter("separation_linear_velocity").value
        )
        map_yaml = str(self.get_parameter("map_yaml").value)
        self.occupancy_map: OccupancyMap | None = (
            OccupancyMap.from_yaml(map_yaml) if map_yaml else None
        )
        if self.occupancy_map is not None:
            self.get_logger().info(
                f"Map-aware following enabled with {map_yaml}, "
                f"clearance={self.map_clearance_m:.2f} m"
            )

        self.states: Dict[str, FollowerState] = {
            namespace: FollowerState() for namespace in self.robot_namespaces
        }
        self.cmd_publishers = {}
        for namespace in self.robot_namespaces:
            if self.pose_topic:
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    self._topic(namespace, self.pose_topic),
                    lambda msg, ns=namespace: self._pose_callback(ns, msg),
                    qos_profile_sensor_data,
                )
            else:
                self.create_subscription(
                    Odometry,
                    self._topic(namespace, self.odom_topic),
                    lambda msg, ns=namespace: self._odom_callback(ns, msg),
                    qos_profile_sensor_data,
                )
            self.create_subscription(
                PoseStamped,
                self._topic(namespace, self.goal_topic),
                lambda msg, ns=namespace: self._goal_callback(ns, msg),
                10,
            )
            self.cmd_publishers[namespace] = []
            for topic in self.cmd_vel_unstamped_topics:
                self.cmd_publishers[namespace].append(
                    (
                        self.create_publisher(
                            Twist, self._topic(namespace, topic), 10
                        ),
                        False,
                    )
                )
            for topic in self.cmd_vel_stamped_topics:
                self.cmd_publishers[namespace].append(
                    (
                        self.create_publisher(
                            TwistStamped, self._topic(namespace, topic), 10
                        ),
                        True,
                    )
                )

        self.create_timer(
            float(self.get_parameter("control_period_sec").value), self._tick
        )

    def _namespaces(self, value) -> list[str]:
        if isinstance(value, str):
            items = value.split(",")
        else:
            items = value
        return [str(item).strip().strip("/") for item in items if str(item).strip()]

    def _topics(self, value, fallback: str, allow_empty: bool = False) -> list[str]:
        if isinstance(value, str):
            items = value.split(",")
        else:
            items = value
        topics = [str(item).strip().strip("/") for item in items if str(item).strip()]
        if not topics and fallback:
            topics = [fallback.strip().strip("/")]
        if not topics and not allow_empty:
            raise ValueError("At least one command velocity topic is required")
        return topics

    def _topic(self, namespace: str, topic: str) -> str:
        return f"/{namespace}/{topic.lstrip('/')}"

    def _odom_callback(self, namespace: str, msg: Odometry) -> None:
        self._update_pose(namespace, msg.pose.pose)

    def _pose_callback(self, namespace: str, msg: PoseWithCovarianceStamped) -> None:
        self._update_pose(namespace, msg.pose.pose)

    def _update_pose(self, namespace: str, pose) -> None:
        q = pose.orientation
        yaw = self._yaw(q.x, q.y, q.z, q.w)
        self.states[namespace].odom = (
            pose.position.x,
            pose.position.y,
            yaw,
        )

    def _goal_callback(self, namespace: str, msg: PoseStamped) -> None:
        self.states[namespace].goal = (msg.pose.position.x, msg.pose.position.y)

    def _tick(self) -> None:
        for namespace, state in self.states.items():
            if state.odom is None or state.goal is None:
                continue
            x, y, yaw = state.odom
            goal_x, goal_y = self._active_goal(namespace, state, x, y)
            if goal_x is None or goal_y is None:
                self._publish(namespace, Twist())
                continue
            dx = goal_x - x
            dy = goal_y - y
            distance = math.hypot(dx, dy)
            cmd = Twist()
            tolerance = (
                self.map_waypoint_tolerance_m
                if self.occupancy_map is not None and state.waypoints
                else self.goal_tolerance_m
            )
            if distance > tolerance:
                desired_yaw = math.atan2(dy, dx)
                heading_error = self._wrap(desired_yaw - yaw)
                cmd.angular.z = self._clamp(
                    self.angular_gain * heading_error,
                    -self.max_angular_velocity,
                    self.max_angular_velocity,
                )
                if abs(heading_error) <= self.heading_tolerance_rad:
                    cmd.linear.x = self._clamp(
                        self.linear_gain * distance,
                        -self.max_linear_velocity,
                        self.max_linear_velocity,
                    )
            cmd = self._apply_robot_spacing(namespace, x, y, yaw, cmd)
            self._publish(namespace, cmd)

    def _active_goal(
        self, namespace: str, state: FollowerState, x: float, y: float
    ) -> Tuple[Optional[float], Optional[float]]:
        if self.occupancy_map is None:
            return state.goal

        now = self.get_clock().now().nanoseconds / 1e9
        goal_changed = state.planned_goal != state.goal
        current_cell = self.occupancy_map.world_to_grid(x, y)
        start_changed = state.planned_from != current_cell and not state.waypoints
        plan_stale = now - state.last_plan_sec >= self.map_replan_period_sec

        if goal_changed or start_changed or (not state.waypoints and plan_stale):
            plan = self.occupancy_map.plan(
                (x, y),
                state.goal,
                self.map_clearance_m,
                self.map_unknown_is_obstacle,
            )
            state.planned_goal = state.goal
            state.planned_from = current_cell
            state.last_plan_sec = now
            if plan is None or not plan.waypoints:
                state.waypoints = []
                self._warn_no_path(namespace, state, now)
                return None, None
            state.waypoints = plan.waypoints[1:] if len(plan.waypoints) > 1 else plan.waypoints

        while state.waypoints:
            waypoint = state.waypoints[0]
            if math.hypot(waypoint[0] - x, waypoint[1] - y) > self.map_waypoint_tolerance_m:
                return waypoint
            state.waypoints.pop(0)

        return state.goal

    def _warn_no_path(self, namespace: str, state: FollowerState, now: float) -> None:
        if now - state.last_warning_sec < 3.0:
            return
        state.last_warning_sec = now
        self.get_logger().warn(
            f"No safe map path for {namespace} to goal {state.goal}; publishing zero velocity"
        )

    def _apply_robot_spacing(
        self, namespace: str, x: float, y: float, yaw: float, cmd: Twist
    ) -> Twist:
        closest: tuple[float, float, float] | None = None
        for other_namespace, other_state in self.states.items():
            if other_namespace == namespace or other_state.odom is None:
                continue
            other_x, other_y, _ = other_state.odom
            distance = math.hypot(x - other_x, y - other_y)
            if closest is None or distance < closest[0]:
                closest = (distance, other_x, other_y)
        if closest is None:
            return cmd

        distance, other_x, other_y = closest
        if distance <= 1e-6:
            away_yaw = yaw + math.pi
        else:
            away_yaw = math.atan2(y - other_y, x - other_x)

        if distance < self.min_robot_separation_m:
            heading_error = self._wrap(away_yaw - yaw)
            avoid = Twist()
            avoid.angular.z = self._clamp(
                self.angular_gain * heading_error,
                -self.max_angular_velocity,
                self.max_angular_velocity,
            )
            if abs(heading_error) <= self.heading_tolerance_rad:
                avoid.linear.x = self.separation_linear_velocity
            return avoid

        if (
            distance < self.slow_robot_separation_m
            and cmd.linear.x > 0.0
            and self.slow_robot_separation_m > self.min_robot_separation_m
        ):
            to_other_x = other_x - x
            to_other_y = other_y - y
            forward_x = math.cos(yaw)
            forward_y = math.sin(yaw)
            moving_toward_other = forward_x * to_other_x + forward_y * to_other_y > 0.0
            if moving_toward_other:
                scale = (distance - self.min_robot_separation_m) / (
                    self.slow_robot_separation_m - self.min_robot_separation_m
                )
                cmd.linear.x *= self._clamp(scale, 0.0, 1.0)
        return cmd

    def _publish(self, namespace: str, cmd: Twist) -> None:
        for publisher, stamped in self.cmd_publishers[namespace]:
            publisher.publish(self._format(cmd, stamped))

    def _format(self, twist: Twist, stamped: bool = False):
        if not stamped:
            return twist
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist = twist
        return msg

    def _yaw(self, x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _wrap(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleGoalFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for publishers in node.cmd_publishers.values():
                for publisher, stamped in publishers:
                    try:
                        publisher.publish(node._format(Twist(), stamped))
                    except Exception:
                        pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
