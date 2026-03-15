import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    is_rover = LaunchConfiguration("rover").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )

    config_filename = (
        "autonomous_to_main.yaml" if is_rover else "main_to_autonomous.yaml"
    )

    config_file = os.path.join(
        get_package_share_directory("perseus_domain_bridge"), "config", config_filename
    )

    domain_bridge_node = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name="domain_bridge",
        output="screen",
        arguments=[config_file],
    )

    return [domain_bridge_node]


def generate_launch_description():
    rover_arg = DeclareLaunchArgument(
        "rover",
        default_value="false",
        description="Set to true when launching on the rover, false for the main station",
    )

    return LaunchDescription(
        [
            rover_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
