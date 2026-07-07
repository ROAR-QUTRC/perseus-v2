# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Relay Isaac AprilTag detections to standard, bridgeable message types.

The zenoh bridge only forwards `/perseus_isaac/*` topics, and we deliberately
keep Isaac-specific message types (isaac_ros_apriltag_interfaces) OFF the wire so
the Humble container and Jazzy host only ever exchange stable, widely-compatible
types. This node subscribes to the Isaac AprilTag output and republishes:

- geometry_msgs/PoseArray      -> /perseus_isaac/apriltag/poses
- vision_msgs/Detection2DArray -> /perseus_isaac/apriltag/detections
- diagnostic_msgs/DiagnosticArray -> /perseus_isaac/health (1 Hz heartbeat)

This module imports isaac_ros_apriltag_interfaces, which only exists inside the
Isaac ROS container image — importing it on the Jazzy host will fail by design.
"""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

# Isaac-specific — present only in the container image (see module docstring).
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray


class AprilTagRelay(Node):
    """Republish Isaac AprilTag detections as standard message types."""

    def __init__(self):
        super().__init__("perseus_isaac_relay")
        self._sub = self.create_subscription(
            AprilTagDetectionArray, "tag_detections", self._on_tags, 10
        )
        self._poses = self.create_publisher(
            PoseArray, "/perseus_isaac/apriltag/poses", 10
        )
        self._dets = self.create_publisher(
            Detection2DArray, "/perseus_isaac/apriltag/detections", 10
        )
        self._health = self.create_publisher(
            DiagnosticArray, "/perseus_isaac/health", 1
        )
        self._relayed = 0
        self.create_timer(1.0, self._on_health)

    def _on_tags(self, msg):
        poses = PoseArray()
        poses.header = msg.header
        dets = Detection2DArray()
        dets.header = msg.header
        for tag in msg.detections:
            pose = _extract_pose(tag)
            poses.poses.append(pose)
            det = Detection2D()
            det.header = msg.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = _tag_class_id(tag)
            hypothesis.hypothesis.score = 1.0
            hypothesis.pose.pose = pose
            det.results.append(hypothesis)
            dets.detections.append(det)
        self._poses.publish(poses)
        self._dets.publish(dets)
        self._relayed += 1

    def _on_health(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "perseus_isaac_relay"
        status.level = DiagnosticStatus.OK
        status.message = "relaying apriltag detections"
        status.values.append(KeyValue(key="relayed", value=str(self._relayed)))
        array.status.append(status)
        self._health.publish(array)


def _extract_pose(tag) -> Pose:
    """Pull a geometry_msgs/Pose out of an AprilTagDetection.

    The pose field layout differs across isaac_ros_apriltag_interfaces versions
    (Pose vs PoseStamped vs PoseWithCovariance*); resolve defensively and confirm
    against the installed version on-device.
    """
    candidate = getattr(tag, "pose", None)
    if candidate is None:
        return Pose()
    inner = getattr(candidate, "pose", candidate)
    return getattr(inner, "pose", inner)


def _tag_class_id(tag) -> str:
    """Format a tag as `<family>:<id>` for the detection class label."""
    family = getattr(tag, "family", "tag")
    tag_id = getattr(tag, "id", 0)
    return f"{family}:{tag_id}"


def main(args=None):
    """Spin the relay node."""
    rclpy.init(args=args)
    node = AprilTagRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
