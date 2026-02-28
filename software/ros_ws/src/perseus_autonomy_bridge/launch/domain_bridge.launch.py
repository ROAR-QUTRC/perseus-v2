"""
domain_bridge.launch.py - DDS Domain Bridge

This launch file starts the domain bridge node to bridge traffic between
different ROS 2 DDS domains.

Usage:
    ros2 launch autonomy domain_bridge.launch.py

    # With custom domains
    ros2 launch autonomy domain_bridge.launch.py source_domain:=0 target_domain:=1

Launch Arguments:
    source_domain  : Source DDS domain ID (default: 0)
    target_domain  : Target DDS domain ID (default: 1)
    use_sim_time   : Use simulated time (default: false)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for domain bridge."""

    # Declare Arguments
    source_domain_arg = DeclareLaunchArgument(
        "source_domain",
        default_value="42",
        description="Source DDS domain ID",
    )
    target_domain_arg = DeclareLaunchArgument(
        "target_domain",
        default_value="10",
        description="Target DDS domain ID",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time if true",
    )

    source_domain = LaunchConfiguration("source_domain")
    target_domain = LaunchConfiguration("target_domain")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Domain bridge node
    domain_bridge_node = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name="domain_bridge",
        output="screen",
        parameters=[
            {"source_domain": source_domain},
            {"target_domain": target_domain},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Create launch description and populate
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(source_domain_arg)
    ld.add_action(target_domain_arg)
    ld.add_action(use_sim_time_arg)

    # Add nodes
    ld.add_action(domain_bridge_node)

    return ld
