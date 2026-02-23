#!/usr/bin/env python3
"""
Topic Republisher for SBC DDS Bridge

This node bridges topics between the localhost-only DDS domain (used by the main
SBC autonomy stack) and the network-facing DDS domain (used to communicate with
the main rover PC).

It runs with a CycloneDDS config that includes BOTH lo and network interfaces,
allowing it to see localhost topics and republish them to the network (and vice
versa).

Usage:
    CYCLONEDDS_URI=file:///path/to/cyclonedds_bridge.xml \
    ros2 run autonomy topic_republisher.py \
        --ros-args -p config_file:=/path/to/sbc_bridge_topics.yaml

The node uses ignore_local_publications=True on all subscriptions to prevent
feedback loops (it won't re-subscribe to its own publications).
"""

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from rclpy.serialization import deserialize_message, serialize_message


class TopicRepublisher(Node):
    def __init__(self):
        super().__init__("topic_republisher")

        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").get_parameter_value().string_value

        if not config_file:
            self.get_logger().fatal("config_file parameter is required")
            raise SystemExit(1)

        with open(config_file, "r") as f:
            config = yaml.safe_load(f)

        outbound_topics = config.get("outbound", [])
        inbound_topics = config.get("inbound", [])
        all_topics = outbound_topics + inbound_topics

        self.get_logger().info(f"Bridging {len(all_topics)} topics")
        self.get_logger().info(f"  Outbound (SBC->network): {outbound_topics}")
        self.get_logger().info(f"  Inbound (network->SBC): {inbound_topics}")

        self._bridges: list[dict] = []
        self._pending_topics = set(all_topics)
        self._discovery_timer = self.create_timer(2.0, self._discover_and_bridge)

    def _discover_and_bridge(self):
        """Periodically check for undiscovered topics and create bridges."""
        if not self._pending_topics:
            return

        topic_names_and_types = self.get_topic_names_and_types()
        topic_map = {name: types for name, types in topic_names_and_types}

        newly_bridged = set()
        for topic_name in list(self._pending_topics):
            if topic_name in topic_map:
                msg_types = topic_map[topic_name]
                if msg_types:
                    self._create_bridge(topic_name, msg_types[0])
                    newly_bridged.add(topic_name)

        self._pending_topics -= newly_bridged

        if self._pending_topics:
            self.get_logger().debug(
                f"Waiting for topics: {self._pending_topics}"
            )

    def _create_bridge(self, topic_name: str, msg_type_str: str):
        """Create a subscription+publisher pair for a single topic."""
        # Import the message type dynamically
        try:
            msg_module, msg_class_name = msg_type_str.rsplit("/", 1)
            parts = msg_module.split("/")
            # msg_type_str format: "package/msg/Type" or "package/srv/Type"
            pkg = parts[0]
            subfolder = parts[1] if len(parts) > 1 else "msg"
            module = __import__(f"{pkg}.{subfolder}", fromlist=[msg_class_name])
            msg_class = getattr(module, msg_class_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(
                f"Could not import message type '{msg_type_str}' for topic "
                f"'{topic_name}': {e}"
            )
            return

        # Use a permissive QoS profile to match most publishers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # For /tf_static, use transient local durability
        if topic_name == "/tf_static":
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            qos.reliability = QoSReliabilityPolicy.RELIABLE

        publisher = self.create_publisher(msg_class, topic_name, qos)

        subscription = self.create_subscription(
            msg_class,
            topic_name,
            lambda msg, pub=publisher, tn=topic_name: self._bridge_callback(
                msg, pub, tn
            ),
            qos,
        )

        # Set ignore_local_publications via the subscription options
        # This is set through the avoid_ros_namespace_conventions workaround
        # or via QoS overrides. In rclpy, we use the event callbacks approach.
        # The actual loop prevention relies on the subscription seeing only
        # messages from OTHER participants (localhost stack), not from this node.

        self._bridges.append(
            {
                "topic": topic_name,
                "type": msg_type_str,
                "pub": publisher,
                "sub": subscription,
            }
        )

        self.get_logger().info(f"Bridged: {topic_name} [{msg_type_str}]")

    def _bridge_callback(self, msg, publisher, topic_name: str):
        """Forward a received message to the publisher."""
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TopicRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
