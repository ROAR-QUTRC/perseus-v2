"""
throttled_bridge.launch.py
==========================
Launch the domain_bridge_throttled executable.

Usage
-----
ros2 launch domain_bridge_throttled throttled_bridge.launch.py \\
    config:=/path/to/bridge_config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value="",
        description=(
            "Absolute path to the throttled bridge YAML config. "
            "Leave empty to use the package default (config/bridge_config.yaml)."
        ),
    )

    def launch_setup(context, *args, **kwargs):
        config = LaunchConfiguration("config").perform(context)

        if not config:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("domain_bridge_throttled")
            config = os.path.join(pkg_share, "config", "bridge_config.yaml")

        return [
            Node(
                package="domain_bridge_throttled",
                executable="throttled_bridge",
                name="throttled_domain_bridge",
                output="screen",
                arguments=[config],
                # Disable rclcpp global init – the node manages its own contexts
                additional_env={"RCUTILS_LOGGING_BUFFERED_STREAM": "1"},
            )
        ]

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=launch_setup),
    ])