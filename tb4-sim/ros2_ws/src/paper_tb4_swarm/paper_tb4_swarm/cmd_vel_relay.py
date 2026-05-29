import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelRelay(Node):
    """Relay Nav2 Jazzy stamped velocity commands to TurtleBot4 unstamped input."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_relay")
        self.declare_parameter("input_topic", "cmd_vel")
        self.declare_parameter("output_topic", "cmd_vel_unstamped")
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(
            TwistStamped,
            self.input_topic,
            self._callback,
            10,
        )
        self.get_logger().info(
            f"Relaying {self.input_topic} TwistStamped to {self.output_topic} Twist"
        )

    def _callback(self, msg: TwistStamped) -> None:
        self.publisher.publish(msg.twist)

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelay()
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
