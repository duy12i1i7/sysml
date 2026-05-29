import random

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class TargetPublisher(Node):
    """Publish a fixed or random target pose."""

    def __init__(self) -> None:
        super().__init__("target_publisher")
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("mode", "fixed")
        self.declare_parameter("target_x", 1.5)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("random_bounds", [-1.8, 1.8, -1.2, 1.2])
        self.declare_parameter("publish_period_sec", 1.0)

        self.mode = str(self.get_parameter("mode").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.target = self._next_target()
        self.publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("target_topic").value), 10
        )
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value), self._tick
        )

    def _next_target(self) -> tuple[float, float]:
        if self.mode == "random":
            xmin, xmax, ymin, ymax = [
                float(value) for value in self.get_parameter("random_bounds").value
            ]
            return random.uniform(xmin, xmax), random.uniform(ymin, ymax)
        return (
            float(self.get_parameter("target_x").value),
            float(self.get_parameter("target_y").value),
        )

    def _tick(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.target[0]
        msg.pose.position.y = self.target[1]
        msg.pose.orientation.w = 1.0
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetPublisher()
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
