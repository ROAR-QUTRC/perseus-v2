"""Launch the whole vision pipeline in one component container.

Both detectors and the overlay run as composable nodes inside a single process with
intra-process comms enabled, so the camera image is handed between them by pointer
instead of being copied and serialised once per subscriber.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Build the launch description for the composed vision pipeline."""
    perseus_vision_dir = get_package_share_directory("perseus_vision")
    config_dir = os.path.join(perseus_vision_dir, "config")
    config_file = os.path.join(config_dir, "perseus_vision.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )

    common_parameters = [
        config_file,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    # extra_arguments enables intra-process comms, which is what makes composing
    # these nodes cheaper than running them as separate processes.
    intra_process = [{"use_intra_process_comms": True}]

    composable_nodes = [
        ComposableNode(
            package="perseus_vision",
            plugin="perseus_vision::ArucoDetector",
            name="aruco_detector",
            parameters=common_parameters,
            extra_arguments=intra_process,
        ),
        ComposableNode(
            package="perseus_vision",
            plugin="perseus_vision::CubeDetector",
            name="cube_detector",
            parameters=common_parameters,
            extra_arguments=intra_process,
        ),
        ComposableNode(
            package="perseus_vision",
            plugin="perseus_vision::DetectionOverlay",
            name="detection_overlay",
            parameters=common_parameters,
            extra_arguments=intra_process,
        ),
    ]

    vision_container = ComposableNodeContainer(
        name="perseus_vision_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            vision_container,
        ]
    )
