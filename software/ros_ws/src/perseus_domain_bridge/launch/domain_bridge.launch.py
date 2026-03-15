from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    config_filename = LaunchConfiguration("config_filename")
    arguments = [
        DeclareLaunchArgument(
            "config_filename",
            default_value="big-brain-bridge.yaml",
            description="Config file for the domain bridge - inside perseus_domain_bridge/config",
        ),
    ]
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("perseus_domain_bridge"),
            "config",
            config_filename,
        ],
    )
    domain_bridge_node = Node(
        executable="domain_bridge",
        package="domain_bridge",
        ros_arguments=["--log-level", "DEBUG"],
        arguments=[
            config_file,
        ],
    )
    nodes = [
        domain_bridge_node,
    ]
    return LaunchDescription(arguments + nodes)
