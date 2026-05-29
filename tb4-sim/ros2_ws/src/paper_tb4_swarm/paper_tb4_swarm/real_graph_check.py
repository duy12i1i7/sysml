import argparse
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class RealGraphCheck(Node):
    def __init__(self, namespaces: list[str], check_motion: bool) -> None:
        super().__init__("real_graph_check")
        self.namespaces = [namespace.strip("/") for namespace in namespaces]
        self.check_motion = check_motion
        self.received: dict[str, bool] = {}
        self.command_topics: dict[str, str] = {}

        for namespace in self.namespaces:
            odom_topic = f"/{namespace}/odom"
            scan_topic = f"/{namespace}/scan"
            self.received[odom_topic] = False
            self.received[scan_topic] = False
            self.create_subscription(
                Odometry,
                odom_topic,
                lambda _msg, topic=odom_topic: self._mark_received(topic),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                LaserScan,
                scan_topic,
                lambda _msg, topic=scan_topic: self._mark_received(topic),
                qos_profile_sensor_data,
            )

    def _mark_received(self, topic: str) -> None:
        self.received[topic] = True

    def refresh_graph(self) -> None:
        topics = {name for name, _types in self.get_topic_names_and_types()}
        for namespace in self.namespaces:
            unstamped = f"/{namespace}/cmd_vel_unstamped"
            stamped = f"/{namespace}/cmd_vel"
            if unstamped in topics:
                self.command_topics[namespace] = unstamped
            elif stamped in topics:
                self.command_topics[namespace] = stamped

    def complete(self) -> bool:
        if not all(self.received.values()):
            return False
        if self.check_motion:
            return all(namespace in self.command_topics for namespace in self.namespaces)
        return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--motion", action="store_true")
    parser.add_argument("--namespaces", nargs="+", default=["robot1", "robot2"])
    args = parser.parse_args(argv)

    rclpy.init()
    node = RealGraphCheck(args.namespaces, args.motion)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.refresh_graph()
            if node.complete():
                break

        missing = False
        for namespace in node.namespaces:
            odom_topic = f"/{namespace}/odom"
            scan_topic = f"/{namespace}/scan"
            if node.received[odom_topic]:
                print(f"OK: {odom_topic} is publishing Odometry")
            else:
                print(
                    f"NO DATA: {odom_topic} did not publish Odometry within {args.timeout:g}s",
                    file=sys.stderr,
                )
                missing = True

            if node.received[scan_topic]:
                print(f"OK: {scan_topic} is publishing LaserScan")
            else:
                print(
                    f"NO DATA: {scan_topic} did not publish LaserScan within {args.timeout:g}s",
                    file=sys.stderr,
                )
                missing = True

        if args.motion:
            for namespace in node.namespaces:
                command_topic = node.command_topics.get(namespace)
                if command_topic:
                    print(f"OK: {command_topic}")
                else:
                    print(
                        f"MISSING: /{namespace}/cmd_vel_unstamped or /{namespace}/cmd_vel",
                        file=sys.stderr,
                    )
                    missing = True

        return 2 if missing else 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
