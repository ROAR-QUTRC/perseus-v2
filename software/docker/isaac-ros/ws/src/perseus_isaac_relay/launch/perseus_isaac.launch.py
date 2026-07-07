# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Composable Isaac ROS perception graph (runs inside the Humble container).

Graph: usb_cam -> isaac_ros image rectify -> isaac_ros AprilTag -> relay, plus a
throttled/compressed debug image. NITROS zero-copy applies between the composable
NITROS nodes inside `perseus_isaac_container`; the camera and relay run as
standalone nodes. Only compact /perseus_isaac/* results leave the container over
the zenoh bridge — never raw imagery except the throttled debug feed.

Object detection (isaac_ros_yolov8 + isaac_ros_tensor_rt) is documented as the
phase-2 addition in docs/source/systems/software/isaac-ros.md: it needs
model-specific tensor names/dimensions that must be set from the cube model
on-device, so it is intentionally not wired blind here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

CAMERA_INFO_URL = "file:///opt/perseus/camera_info/default_camera.yaml"


def generate_launch_description():
    """Build the Isaac ROS perception launch description."""
    pkg = get_package_share_directory("perseus_isaac_relay")
    params = os.path.join(pkg, "config", "perseus_isaac.yaml")

    camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        parameters=[params, {"camera_info_url": CAMERA_INFO_URL}],
        remappings=[
            ("image_raw", "/image_raw"),
            ("camera_info", "/camera_info"),
        ],
    )

    container = ComposableNodeContainer(
        name="perseus_isaac_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="isaac_ros_image_proc",
                plugin="nvidia::isaac_ros::image_proc::RectifyNode",
                name="rectify",
                parameters=[params],
                remappings=[
                    ("image_raw", "/image_raw"),
                    ("camera_info", "/camera_info"),
                    ("image_rect", "/image_rect"),
                    ("camera_info_rect", "/camera_info_rect"),
                ],
            ),
            ComposableNode(
                package="isaac_ros_apriltag",
                plugin="nvidia::isaac_ros::apriltag::AprilTagNode",
                name="apriltag",
                parameters=[params],
                remappings=[
                    ("image", "/image_rect"),
                    ("camera_info", "/camera_info_rect"),
                ],
            ),
        ],
        output="screen",
    )

    relay = Node(
        package="perseus_isaac_relay",
        executable="relay",
        name="perseus_isaac_relay",
        remappings=[("tag_detections", "/tag_detections")],
    )

    # Throttle the rectified image to 2 Hz then republish it compressed, so an
    # operator can peek over the bridge without flooding it with raw frames.
    debug_throttle = Node(
        package="topic_tools",
        executable="throttle",
        name="debug_image_throttle",
        arguments=[
            "messages",
            "/image_rect",
            "2.0",
            "/perseus_isaac/debug/image",
        ],
    )
    debug_compress = Node(
        package="image_transport",
        executable="republish",
        name="debug_image_compress",
        arguments=["raw", "compressed"],
        remappings=[
            ("in", "/perseus_isaac/debug/image"),
            ("out/compressed", "/perseus_isaac/debug/image/compressed"),
        ],
    )

    return LaunchDescription([camera, container, relay, debug_throttle, debug_compress])
