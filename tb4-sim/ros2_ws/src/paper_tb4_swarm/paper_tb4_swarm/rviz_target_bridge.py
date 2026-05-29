from typing import Iterable

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node


class RvizTargetBridge(Node):
    """Convert RViz operator clicks into the shared swarm target topic."""

    def __init__(self) -> None:
        super().__init__("rviz_target_bridge")
        self.declare_parameter("goal_pose_namespaces", [""])
        self.declare_parameter("clicked_point_namespaces", ["", "robot1", "robot2"])
        self.declare_parameter("goal_pose_topic", "goal_pose")
        self.declare_parameter("clicked_point_topic", "clicked_point")
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("use_goal_pose", True)
        self.declare_parameter("use_clicked_point", True)

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.target_publisher = self.create_publisher(
            PoseStamped, self.target_topic, 10
        )

        goal_namespaces = self._namespaces(
            self.get_parameter("goal_pose_namespaces").value
        )
        clicked_namespaces = self._namespaces(
            self.get_parameter("clicked_point_namespaces").value
        )
        goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        clicked_point_topic = str(self.get_parameter("clicked_point_topic").value)

        if bool(self.get_parameter("use_goal_pose").value):
            for namespace in goal_namespaces:
                self.create_subscription(
                    PoseStamped,
                    self._topic(namespace, goal_pose_topic),
                    self._goal_callback,
                    10,
                )

        if bool(self.get_parameter("use_clicked_point").value):
            for namespace in clicked_namespaces:
                self.create_subscription(
                    PointStamped,
                    self._topic(namespace, clicked_point_topic),
                    self._point_callback,
                    10,
                )

        watched = ", ".join(
            sorted(
                {
                    self._topic(namespace, goal_pose_topic)
                    for namespace in goal_namespaces
                }
                | {
                    self._topic(namespace, clicked_point_topic)
                    for namespace in clicked_namespaces
                }
            )
        )
        self.get_logger().info(
            f"RViz target bridge publishing {self.target_topic}; watching {watched}"
        )

    def _namespaces(self, value) -> list[str]:
        if isinstance(value, str):
            items: Iterable = value.split(",")
        else:
            items = value
        namespaces = [str(item).strip().strip("/") for item in items]
        return namespaces or [""]

    def _topic(self, namespace: str, topic: str) -> str:
        topic = topic.strip().strip("/")
        namespace = namespace.strip().strip("/")
        if not namespace:
            return f"/{topic}"
        return f"/{namespace}/{topic}"

    def _goal_callback(self, msg: PoseStamped) -> None:
        target = PoseStamped()
        target.header = msg.header
        target.header.stamp = self.get_clock().now().to_msg()
        if not target.header.frame_id:
            target.header.frame_id = self.target_frame
        target.pose = msg.pose
        self.target_publisher.publish(target)
        self.get_logger().info(
            f"RViz goal -> {self.target_topic}: "
            f"x={target.pose.position.x:.2f}, y={target.pose.position.y:.2f}"
        )

    def _point_callback(self, msg: PointStamped) -> None:
        target = PoseStamped()
        target.header = msg.header
        target.header.stamp = self.get_clock().now().to_msg()
        if not target.header.frame_id:
            target.header.frame_id = self.target_frame
        target.pose.position.x = msg.point.x
        target.pose.position.y = msg.point.y
        target.pose.position.z = msg.point.z
        target.pose.orientation.w = 1.0
        self.target_publisher.publish(target)
        self.get_logger().info(
            f"RViz point -> {self.target_topic}: "
            f"x={target.pose.position.x:.2f}, y={target.pose.position.y:.2f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizTargetBridge()
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
