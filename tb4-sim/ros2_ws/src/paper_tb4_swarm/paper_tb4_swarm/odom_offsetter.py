import math
from copy import deepcopy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OdomOffsetter(Node):
    """Convert per-model Gazebo odometry into a shared map frame."""

    def __init__(self) -> None:
        super().__init__("odom_offsetter")
        self.declare_parameter("robot_namespaces", ["robot1", "robot2"])
        self.declare_parameter("input_odom_topic", "odom_raw")
        self.declare_parameter("output_odom_topic", "odom")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("initial_pose_x", [0.0, 0.0])
        self.declare_parameter("initial_pose_y", [0.0, 1.0])
        self.declare_parameter("initial_pose_yaw", [0.0, 0.0])
        self.declare_parameter("max_pose_jump_m", 0.35)

        self.robot_namespaces = self._namespaces(
            self.get_parameter("robot_namespaces").value
        )
        self.input_odom_topic = str(self.get_parameter("input_odom_topic").value)
        self.output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.initial_x = [float(v) for v in self.get_parameter("initial_pose_x").value]
        self.initial_y = [float(v) for v in self.get_parameter("initial_pose_y").value]
        self.initial_yaw = [
            float(v) for v in self.get_parameter("initial_pose_yaw").value
        ]
        self.max_pose_jump_m = float(self.get_parameter("max_pose_jump_m").value)
        self.odom_publishers = {}
        self.last_xy = {}
        for namespace in self.robot_namespaces:
            self.odom_publishers[namespace] = self.create_publisher(
                Odometry, self._topic(namespace, self.output_odom_topic), 10
            )
            self.create_subscription(
                Odometry,
                self._topic(namespace, self.input_odom_topic),
                lambda msg, ns=namespace: self._callback(ns, msg),
                qos_profile_sensor_data,
            )

    def _namespaces(self, value) -> list[str]:
        if isinstance(value, str):
            items = value.split(",")
        else:
            items = value
        return [str(item).strip().strip("/") for item in items if str(item).strip()]

    def _topic(self, namespace: str, topic: str) -> str:
        return f"/{namespace}/{topic.lstrip('/')}"

    def _callback(self, namespace: str, msg: Odometry) -> None:
        index = self.robot_namespaces.index(namespace)
        yaw = self.initial_yaw[index] if index < len(self.initial_yaw) else 0.0
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = msg.pose.pose.position.x
        local_y = msg.pose.pose.position.y
        out = Odometry()
        out.header = deepcopy(msg.header)
        out.header.frame_id = self.frame_id
        out.child_frame_id = f"{namespace}/base_link"
        out.pose = deepcopy(msg.pose)
        out.twist = deepcopy(msg.twist)
        out.pose.pose.position.x = (
            self.initial_x[index] + cos_yaw * local_x - sin_yaw * local_y
        )
        out.pose.pose.position.y = (
            self.initial_y[index] + sin_yaw * local_x + cos_yaw * local_y
        )
        xy = (out.pose.pose.position.x, out.pose.pose.position.y)
        previous_xy = self.last_xy.get(namespace)
        if previous_xy is not None:
            jump = math.hypot(xy[0] - previous_xy[0], xy[1] - previous_xy[1])
            if jump > self.max_pose_jump_m:
                self.get_logger().warn(
                    f"Dropping implausible {namespace} odom jump: {jump:.3f} m",
                    throttle_duration_sec=2.0,
                )
                return
        self.last_xy[namespace] = xy
        self.odom_publishers[namespace].publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomOffsetter()
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
