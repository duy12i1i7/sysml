import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from typing import Optional


class OdomTfBroadcaster(Node):
    """Publish odom -> base_link TF from TurtleBot4 odometry messages."""

    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("parent_frame", "")
        self.declare_parameter("child_frame", "")
        self.declare_parameter("publish_rate_hz", 20.0)

        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        self.broadcaster = TransformBroadcaster(self)
        self.latest_transform: Optional[TransformStamped] = None
        self._received_first_odom = False
        reliable_odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            reliable_odom_qos,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / publish_rate_hz, self._publish_latest)
        self.get_logger().info(f"Publishing TF from {odom_topic} odometry")

    def _odom_callback(self, msg: Odometry) -> None:
        if not self._received_first_odom:
            self._received_first_odom = True
            self.get_logger().info(
                f"Received odometry; publishing {self.parent_frame or msg.header.frame_id} "
                f"-> {self.child_frame or msg.child_frame_id} TF"
            )
        transform = TransformStamped()
        transform.header = msg.header
        if self.parent_frame:
            transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame or msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.latest_transform = transform
        self._publish_latest()

    def _publish_latest(self) -> None:
        if self.latest_transform is None:
            return

        transform = TransformStamped()
        transform.header = self.latest_transform.header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.child_frame_id = self.latest_transform.child_frame_id
        transform.transform = self.latest_transform.transform
        self.broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
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
