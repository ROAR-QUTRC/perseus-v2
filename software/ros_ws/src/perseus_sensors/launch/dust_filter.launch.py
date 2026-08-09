"""Launch the dust filter node with its default configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the dust filter node."""
    config_file = os.path.join(
        get_package_share_directory("perseus_sensors"),
        "config",
        "dust_filter.yaml",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )

    dust_filter_node = Node(
        package="perseus_sensors",
        executable="dust_filter",
        name="dust_filter",
        parameters=[config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            dust_filter_node,
        ]
    )
