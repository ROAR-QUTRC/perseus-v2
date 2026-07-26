"""Launch the detection overlay node on its own, against already-running detectors."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the detection overlay node."""
    perseus_vision_dir = get_package_share_directory("perseus_vision")
    config_dir = os.path.join(perseus_vision_dir, "config")
    config_file = os.path.join(config_dir, "perseus_vision.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )

    detection_overlay_node = Node(
        package="perseus_vision",
        executable="detection_overlay_node",
        name="detection_overlay",
        parameters=[config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            detection_overlay_node,
        ]
    )
