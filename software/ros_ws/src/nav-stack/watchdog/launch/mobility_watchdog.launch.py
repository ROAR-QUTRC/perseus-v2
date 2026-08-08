"""Launch the mobility watchdog node with its default configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the mobility watchdog node."""
    watchdog_dir = get_package_share_directory("watchdog")
    config_file = os.path.join(watchdog_dir, "config", "watchdog.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )

    mobility_watchdog_node = Node(
        package="watchdog",
        executable="mobility_watchdog",
        name="mobility_watchdog",
        parameters=[config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            mobility_watchdog_node,
        ]
    )
