import math
from statistics import median
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class WallFollowMapper(Node):
    """Conservative lidar wall-following mapper for narrow real spaces."""

    def __init__(self) -> None:
        super().__init__("wall_follow_mapper")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("cmd_vel_topic", "cmd_vel_unstamped")
        self.declare_parameter("scan_yaw_offset_deg", 0.0)
        self.declare_parameter(
            "duration_sec", 180.0, ParameterDescriptor(dynamic_typing=True)
        )
        self.declare_parameter(
            "initial_spin_sec", 22.0, ParameterDescriptor(dynamic_typing=True)
        )
        self.declare_parameter("target_wall_distance_m", 0.65)
        self.declare_parameter("front_stop_distance_m", 0.72)
        self.declare_parameter("emergency_stop_distance_m", 0.34)
        self.declare_parameter("linear_speed_mps", 0.055)
        self.declare_parameter("turn_speed_radps", 0.34)
        self.declare_parameter("control_period_sec", 0.1)

        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.initial_spin_sec = float(self.get_parameter("initial_spin_sec").value)
        self.scan_yaw_offset_deg = float(
            self.get_parameter("scan_yaw_offset_deg").value
        )
        self.target_wall_distance_m = float(
            self.get_parameter("target_wall_distance_m").value
        )
        self.front_stop_distance_m = float(
            self.get_parameter("front_stop_distance_m").value
        )
        self.emergency_stop_distance_m = float(
            self.get_parameter("emergency_stop_distance_m").value
        )
        self.linear_speed_mps = float(self.get_parameter("linear_speed_mps").value)
        self.turn_speed_radps = float(self.get_parameter("turn_speed_radps").value)
        self.start_time = self.get_clock().now().nanoseconds * 1.0e-9
        self.last_scan: Optional[LaserScan] = None
        self.finished = False

        self.publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(
            float(self.get_parameter("control_period_sec").value),
            self._tick,
        )
        self.get_logger().info(
            "Wall-follow mapper active: "
            f"duration={self.duration_sec:.0f}s, "
            f"target_wall={self.target_wall_distance_m:.2f}m, "
            f"scan_yaw_offset={self.scan_yaw_offset_deg:.1f}deg"
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _tick(self) -> None:
        if self.finished:
            return

        elapsed = self.get_clock().now().nanoseconds * 1.0e-9 - self.start_time
        if elapsed >= self.duration_sec:
            self._finish()
            return

        cmd = Twist()
        if self.last_scan is None:
            self.publisher.publish(cmd)
            self.get_logger().warn("Waiting for lidar scan", throttle_duration_sec=3.0)
            return

        front = self._body_sector_median(-18.0, 18.0)
        front_left = self._body_sector_median(18.0, 55.0)
        front_right = self._body_sector_median(-55.0, -18.0)
        left = self._body_sector_median(65.0, 105.0)
        right = self._body_sector_median(-105.0, -65.0)

        if elapsed < self.initial_spin_sec:
            cmd.angular.z = self.turn_speed_radps * 0.65
            self.publisher.publish(cmd)
            return

        if front is None or right is None:
            cmd.angular.z = self.turn_speed_radps * 0.45
            self.publisher.publish(cmd)
            return

        nearest_front = min(
            value
            for value in [front, front_left, front_right]
            if value is not None
        )
        front_right_close = front_right is not None and front_right < 0.46
        turn_direction = self._open_turn_direction(left, right, front_left, front_right)
        if nearest_front < self.emergency_stop_distance_m:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed_radps * turn_direction
            self.get_logger().warn(
                "Tight obstacle ahead; rotating toward open space",
                throttle_duration_sec=3.0,
            )
        elif front < self.front_stop_distance_m or front_right_close:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed_radps * turn_direction
        elif right > 2.0:
            cmd.linear.x = self.linear_speed_mps * 0.65
            cmd.angular.z = -0.16
        else:
            error = self.target_wall_distance_m - right
            cmd.linear.x = self.linear_speed_mps
            cmd.angular.z = self._clamp(0.9 * error, -0.26, 0.26)

        self.publisher.publish(cmd)

    def _open_turn_direction(
        self,
        left: Optional[float],
        right: Optional[float],
        front_left: Optional[float],
        front_right: Optional[float],
    ) -> float:
        left_score = self._space_score(left) + 0.5 * self._space_score(front_left)
        right_score = self._space_score(right) + 0.5 * self._space_score(front_right)
        return 1.0 if left_score >= right_score else -1.0

    def _space_score(self, value: Optional[float]) -> float:
        return 0.0 if value is None else min(value, 3.0)

    def _body_sector_median(self, min_deg: float, max_deg: float) -> Optional[float]:
        assert self.last_scan is not None
        values = []
        for angle_deg in self._angle_range(min_deg, max_deg, 3.0):
            value = self._range_at_body_angle(angle_deg)
            if value is not None:
                values.append(value)
        if not values:
            return None
        return median(values)

    def _sector_median(self, min_deg: float, max_deg: float) -> Optional[float]:
        assert self.last_scan is not None
        values = []
        for angle_deg in self._angle_range(min_deg, max_deg, 3.0):
            value = self._range_at(math.radians(angle_deg))
            if value is not None:
                values.append(value)
        if not values:
            return None
        return median(values)

    def _range_at_body_angle(self, angle_deg: float) -> Optional[float]:
        scan_angle_deg = angle_deg + self.scan_yaw_offset_deg
        return self._range_at(math.radians(scan_angle_deg))

    def _angle_range(self, min_deg: float, max_deg: float, step_deg: float):
        angle = min_deg
        while angle <= max_deg:
            yield angle
            angle += step_deg

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

    def _finish(self) -> None:
        self.finished = True
        self.publisher.publish(Twist())
        self.get_logger().info("Wall-follow mapping duration completed; stopping robot.")

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallFollowMapper()
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
