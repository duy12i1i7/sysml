import math
import time
from dataclasses import dataclass
from statistics import median
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


@dataclass
class HeadingCandidate:
    angle_deg: float
    front_m: float
    center_m: float
    side_m: float
    score: float


class ClearanceExplorer(Node):
    """Local coverage bootstrapper for a physical TurtleBot4.

    This node deliberately avoids reverse motion and global planning. It first
    scans in place, then repeats: pick the clearest body-frame corridor, align,
    creep forward one short odometry step, stop, and resurvey. That makes it
    usable before a SLAM map is large enough for Nav2 frontier planning.
    """

    def __init__(self) -> None:
        super().__init__("clearance_explorer")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("hazard_topic", "hazard_detection")
        self.declare_parameter("cmd_vel_topic", "cmd_vel_unstamped")
        self.declare_parameter("scan_yaw_offset_deg", 0.0)
        self.declare_parameter(
            "duration_sec", 600.0, ParameterDescriptor(dynamic_typing=True)
        )
        self.declare_parameter(
            "initial_spin_sec", 0.0, ParameterDescriptor(dynamic_typing=True)
        )
        self.declare_parameter("initial_scan_turn_rad", 6.4)
        self.declare_parameter("rescan_turn_rad", 1.05)
        self.declare_parameter("drive_step_m", 0.18)
        self.declare_parameter("max_drive_step_sec", 18.0)
        self.declare_parameter("linear_speed_mps", 0.08)
        self.declare_parameter("turn_speed_radps", 0.24)
        self.declare_parameter("front_clearance_m", 0.42)
        self.declare_parameter("side_clearance_m", 0.18)
        self.declare_parameter("emergency_stop_distance_m", 0.20)
        self.declare_parameter("hazard_hold_sec", 1.2)
        self.declare_parameter("hazard_turn_sec", 3.0)
        self.declare_parameter("turn_heading_tolerance_deg", 18.0)
        self.declare_parameter("stuck_window_sec", 12.0)
        self.declare_parameter("stuck_distance_m", 0.012)
        self.declare_parameter("control_period_sec", 0.1)

        self.scan_yaw_offset_deg = float(
            self.get_parameter("scan_yaw_offset_deg").value
        )
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.initial_spin_sec = float(self.get_parameter("initial_spin_sec").value)
        self.initial_scan_turn_rad = float(
            self.get_parameter("initial_scan_turn_rad").value
        )
        self.rescan_turn_rad = float(self.get_parameter("rescan_turn_rad").value)
        self.drive_step_m = float(self.get_parameter("drive_step_m").value)
        self.max_drive_step_sec = float(
            self.get_parameter("max_drive_step_sec").value
        )
        self.linear_speed_mps = float(self.get_parameter("linear_speed_mps").value)
        self.turn_speed_radps = float(self.get_parameter("turn_speed_radps").value)
        self.front_clearance_m = float(self.get_parameter("front_clearance_m").value)
        self.side_clearance_m = float(self.get_parameter("side_clearance_m").value)
        self.emergency_stop_distance_m = float(
            self.get_parameter("emergency_stop_distance_m").value
        )
        self.hazard_hold_sec = float(self.get_parameter("hazard_hold_sec").value)
        self.hazard_turn_sec = float(self.get_parameter("hazard_turn_sec").value)
        self.turn_heading_tolerance_deg = float(
            self.get_parameter("turn_heading_tolerance_deg").value
        )
        self.stuck_window_sec = float(self.get_parameter("stuck_window_sec").value)
        self.stuck_distance_m = float(self.get_parameter("stuck_distance_m").value)

        self.start_time = self._now_sec()
        self.mode = "survey"
        self.mode_started = self.start_time
        self.yaw_since_mode = 0.0
        self.last_yaw: Optional[float] = None
        self.last_scan: Optional[LaserScan] = None
        self.latest_pose: Optional[tuple[float, float, float]] = None
        self.drive_start_time: Optional[float] = None
        self.drive_start_pose: Optional[tuple[float, float]] = None
        self.recovery_sign = 1.0
        self.last_hazard_text = ""
        self.hazard_hold_until = 0.0
        self.hazard_turn_until = 0.0
        self.finished = False

        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            HazardDetectionVector,
            str(self.get_parameter("hazard_topic").value),
            self._hazard_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(
            float(self.get_parameter("control_period_sec").value), self._tick
        )
        self.get_logger().info(
            "Adaptive clearance explorer active: "
            f"duration={self.duration_sec:.0f}s, step={self.drive_step_m:.2f}m, "
            f"front_clearance={self.front_clearance_m:.2f}m, "
            f"side_clearance={self.side_clearance_m:.2f}m, "
            f"scan_yaw_offset={self.scan_yaw_offset_deg:.1f}deg"
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _odom_callback(self, msg: Odometry) -> None:
        point = msg.pose.pose.position
        yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        if self.last_yaw is not None:
            self.yaw_since_mode += abs(self._angle_delta(yaw, self.last_yaw))
        self.last_yaw = yaw
        self.latest_pose = (point.x, point.y, yaw)

    def _hazard_callback(self, msg: HazardDetectionVector) -> None:
        active = [
            detection for detection in msg.detections
            if detection.type in {
                HazardDetection.BUMP,
                HazardDetection.CLIFF,
                HazardDetection.STALL,
                HazardDetection.WHEEL_DROP,
                HazardDetection.OBJECT_PROXIMITY,
            }
        ]
        if not active:
            return
        now = self._now_sec()
        self.hazard_hold_until = max(self.hazard_hold_until, now + self.hazard_hold_sec)
        self.hazard_turn_until = max(
            self.hazard_turn_until, now + self.hazard_hold_sec + self.hazard_turn_sec
        )
        self.last_hazard_text = ",".join(self._hazard_name(item.type) for item in active)

    def _tick(self) -> None:
        if self.finished:
            return

        now = self._now_sec()
        if now - self.start_time >= self.duration_sec:
            self._finish()
            return

        cmd = Twist()
        if self.last_scan is None:
            self.publisher.publish(cmd)
            self.get_logger().warn("Waiting for lidar scan", throttle_duration_sec=3.0)
            return

        if self.latest_pose is None:
            self.publisher.publish(cmd)
            self.get_logger().warn("Waiting for odometry", throttle_duration_sec=3.0)
            return

        if now < self.hazard_hold_until:
            self._reset_drive()
            self.publisher.publish(cmd)
            self.get_logger().warn(
                f"Hazard {self.last_hazard_text}; holding stop",
                throttle_duration_sec=1.0,
            )
            return

        if now < self.hazard_turn_until:
            self._reset_drive()
            cmd.angular.z = self.turn_speed_radps * self.recovery_sign
            self.publisher.publish(cmd)
            self.get_logger().warn(
                f"Hazard {self.last_hazard_text}; rotating away",
                throttle_duration_sec=1.0,
            )
            return

        if self.mode == "survey":
            cmd = self._survey_command(now)
        elif self.mode == "align":
            cmd = self._align_command()
        elif self.mode == "drive":
            cmd = self._drive_command(now)
        elif self.mode == "recovery":
            cmd = self._recovery_command()
        else:
            self._set_mode("survey")
            cmd.angular.z = self.turn_speed_radps * self.recovery_sign

        self.publisher.publish(cmd)
        self._log_status(cmd)

    def _survey_command(self, now: float) -> Twist:
        cmd = Twist()
        cmd.angular.z = self.turn_speed_radps * self.recovery_sign
        time_done = self.initial_spin_sec > 0.0 and (
            now - self.mode_started >= self.initial_spin_sec
        )
        yaw_done = self.yaw_since_mode >= self.initial_scan_turn_rad
        if time_done or yaw_done:
            self._set_mode("align")
            cmd.angular.z = 0.0
        return cmd

    def _align_command(self) -> Twist:
        cmd = Twist()
        candidate = self._best_heading()
        if candidate is None:
            self._set_mode("recovery")
            cmd.angular.z = self.turn_speed_radps * self.recovery_sign
            return cmd

        self.recovery_sign = self._turn_sign(candidate.angle_deg)
        if abs(candidate.angle_deg) <= self.turn_heading_tolerance_deg:
            self._start_drive()
            return cmd

        cmd.angular.z = self.turn_speed_radps * self._turn_sign(candidate.angle_deg)
        return cmd

    def _drive_command(self, now: float) -> Twist:
        cmd = Twist()
        if not self._front_is_safe():
            self._reset_drive()
            self._set_mode("align")
            return cmd

        distance = self._drive_distance()
        elapsed = now - (self.drive_start_time or now)
        if distance >= self.drive_step_m or elapsed >= self.max_drive_step_sec:
            self._reset_drive()
            self._set_mode("align")
            return cmd

        if elapsed >= self.stuck_window_sec and distance < self.stuck_distance_m:
            self._reset_drive()
            self._set_mode("recovery")
            self.get_logger().warn(
                "Low odometry progress during forward step; resurveying",
                throttle_duration_sec=2.0,
            )
            return cmd

        candidate = self._best_heading()
        steer_deg = 0.0 if candidate is None else candidate.angle_deg
        if abs(steer_deg) > 35.0:
            self._reset_drive()
            self._set_mode("align")
            return cmd

        cmd.linear.x = self.linear_speed_mps
        cmd.angular.z = self._clamp(math.radians(steer_deg) * 0.65, -0.14, 0.14)
        return cmd

    def _recovery_command(self) -> Twist:
        cmd = Twist()
        cmd.angular.z = self.turn_speed_radps * self.recovery_sign
        if self.yaw_since_mode >= self.rescan_turn_rad:
            self._set_mode("align")
            cmd.angular.z = 0.0
        return cmd

    def _start_drive(self) -> None:
        assert self.latest_pose is not None
        self._set_mode("drive")
        self.drive_start_time = self._now_sec()
        self.drive_start_pose = (self.latest_pose[0], self.latest_pose[1])

    def _reset_drive(self) -> None:
        self.drive_start_time = None
        self.drive_start_pose = None

    def _drive_distance(self) -> float:
        if self.latest_pose is None or self.drive_start_pose is None:
            return 0.0
        return math.hypot(
            self.latest_pose[0] - self.drive_start_pose[0],
            self.latest_pose[1] - self.drive_start_pose[1],
        )

    def _set_mode(self, mode: str) -> None:
        if mode == self.mode:
            return
        self.mode = mode
        self.mode_started = self._now_sec()
        self.yaw_since_mode = 0.0

    def _best_heading(self) -> Optional[HeadingCandidate]:
        best: Optional[HeadingCandidate] = None
        fallback: Optional[HeadingCandidate] = None
        for angle_deg in range(-170, 171, 5):
            candidate = self._score_heading(float(angle_deg))
            if candidate is None:
                continue
            if fallback is None or candidate.score > fallback.score:
                fallback = candidate
            if not self._heading_is_driveable(candidate):
                continue
            if best is None or candidate.score > best.score:
                best = candidate

        if best is not None:
            return best
        if fallback is not None:
            self.recovery_sign = self._turn_sign(fallback.angle_deg)
        return None

    def _score_heading(self, angle_deg: float) -> Optional[HeadingCandidate]:
        center = self._body_sector_value(angle_deg - 7.0, angle_deg + 7.0, "min")
        front = self._body_sector_value(angle_deg - 18.0, angle_deg + 18.0, "p20")
        left = self._body_sector_value(angle_deg + 20.0, angle_deg + 50.0, "p20")
        right = self._body_sector_value(angle_deg - 50.0, angle_deg - 20.0, "p20")
        values = [value for value in (center, front, left, right) if value is not None]
        if center is None or front is None or not values:
            return None
        side_values = [value for value in (left, right) if value is not None]
        side = min(side_values) if side_values else front

        forward_bonus = 0.18 if abs(angle_deg) <= 15.0 else 0.0
        angle_cost = 0.0035 * abs(angle_deg)
        score = min(front, 2.0) + 0.25 * min(side, 1.2) + forward_bonus - angle_cost
        return HeadingCandidate(
            angle_deg=angle_deg,
            front_m=front,
            center_m=center,
            side_m=side,
            score=score,
        )

    def _heading_is_driveable(self, candidate: HeadingCandidate) -> bool:
        return (
            candidate.center_m > self.emergency_stop_distance_m + 0.05
            and candidate.front_m > self.front_clearance_m
            and candidate.side_m > self.side_clearance_m
        )

    def _front_is_safe(self) -> bool:
        direct = self._body_sector_value(-8.0, 8.0, "min")
        front = self._body_sector_value(-20.0, 20.0, "p20")
        if direct is None or front is None:
            return False
        return (
            direct > self.emergency_stop_distance_m
            and front > self.emergency_stop_distance_m + 0.10
        )

    def _body_sector_value(
        self, min_deg: float, max_deg: float, mode: str
    ) -> Optional[float]:
        assert self.last_scan is not None
        values = []
        angle = min_deg
        while angle <= max_deg:
            value = self._range_at_body_angle(angle)
            if value is not None:
                values.append(value)
            angle += 3.0
        if not values:
            return None
        values.sort()
        if mode == "min":
            return values[0]
        if mode == "p20":
            return values[min(len(values) - 1, max(0, int(0.2 * len(values))))]
        return median(values)

    def _range_at_body_angle(self, angle_deg: float) -> Optional[float]:
        scan_angle_deg = angle_deg + self.scan_yaw_offset_deg
        return self._range_at(math.radians(scan_angle_deg))

    def _range_at(self, angle: float) -> Optional[float]:
        assert self.last_scan is not None
        msg = self.last_scan
        while angle < msg.angle_min:
            angle += 2.0 * math.pi
        while angle > msg.angle_max:
            angle -= 2.0 * math.pi
        index = round((angle - msg.angle_min) / msg.angle_increment)
        if index < 0 or index >= len(msg.ranges):
            return None
        value = float(msg.ranges[index])
        if not math.isfinite(value):
            return None
        if value < msg.range_min or value > msg.range_max:
            return None
        return value

    def _log_status(self, cmd: Twist) -> None:
        direct = self._body_sector_value(-8.0, 8.0, "min")
        front = self._body_sector_value(-20.0, 20.0, "p20")
        candidate = self._best_heading()
        candidate_text = "none"
        if candidate is not None:
            candidate_text = (
                f"{candidate.angle_deg:.0f}deg/"
                f"{candidate.front_m:.2f}m/{candidate.side_m:.2f}m"
            )
        self.get_logger().info(
            f"mode={self.mode} yaw={self.yaw_since_mode:.2f} "
            f"front={self._fmt(front)} direct={self._fmt(direct)} "
            f"best={candidate_text} cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})",
            throttle_duration_sec=3.0,
        )

    def _hazard_name(self, hazard_type: int) -> str:
        names = {
            HazardDetection.BACKUP_LIMIT: "backup_limit",
            HazardDetection.BUMP: "bump",
            HazardDetection.CLIFF: "cliff",
            HazardDetection.STALL: "stall",
            HazardDetection.WHEEL_DROP: "wheel_drop",
            HazardDetection.OBJECT_PROXIMITY: "object_proximity",
        }
        return names.get(hazard_type, str(hazard_type))

    def _finish(self) -> None:
        self.finished = True
        self.publisher.publish(Twist())
        self.get_logger().info("Clearance exploration duration completed; stopping robot.")

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())

    def _fmt(self, value: Optional[float]) -> str:
        return "None" if value is None else f"{value:.2f}m"

    def _now_sec(self) -> float:
        return time.monotonic()

    def _turn_sign(self, angle_deg: float) -> float:
        return 1.0 if angle_deg >= 0.0 else -1.0

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _angle_delta(self, current: float, previous: float) -> float:
        return math.atan2(math.sin(current - previous), math.cos(current - previous))

    def _yaw_from_quaternion(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClearanceExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
