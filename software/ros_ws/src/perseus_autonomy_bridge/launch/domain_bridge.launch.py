"""
domain_bridge.launch.py - DDS Domain Bridge

This launch file starts the domain bridge node to bridge traffic between
different ROS 2 DDS domains.

Usage:
    ros2 launch perseus_autonomy_bridge domain_bridge.launch.py

    # With custom domains
    ros2 launch perseus_autonomy_bridge domain_bridge.launch.py from_domain:=42 to_domain:=10

Launch Arguments:
    from_domain    : Source DDS domain ID (default: 42)
    to_domain      : Target DDS domain ID (default: 10)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for domain bridge."""
    
    # Declare Arguments
    from_domain_arg = DeclareLaunchArgument(
        "from_domain",
        default_value="42",
        description="Source DDS domain ID",
    )
    to_domain_arg = DeclareLaunchArgument(
        "to_domain",
        default_value="10",
        description="Target DDS domain ID",
    )

    from_domain = LaunchConfiguration("from_domain")
    to_domain = LaunchConfiguration("to_domain")

    # Get the config file path
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("perseus_autonomy_bridge"),
            "config",
            "domain_bridge_config.yaml",
        ]
    )

    # Domain bridge executable
    domain_bridge_node = ExecuteProcess(
        cmd=[
            "domain_bridge",
            config_file,
            "--from",
            from_domain,
            "--to",
            to_domain,
        ],
        output="screen",
    )

    # Create launch description and populate
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(from_domain_arg)
    ld.add_action(to_domain_arg)
