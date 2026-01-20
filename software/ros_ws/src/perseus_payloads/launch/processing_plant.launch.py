from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node


def generate_launch_description():
    # ARGUMENTS
    i2c_bus = LaunchConfiguration("i2c_bus", default="/dev/i2c-0")
    arguments = [
        DeclareLaunchArgument(
            "i2c_bus",
            default_value="/dev/i2c-0",
            description="i2c bus: this is usually '/dev/i2c-*' where '*' is a number",
        ),
    ]

    # NODES
    sensors = Node(
        package="perseus_payloads",
        executable="sensors",
        parameters=[
            {
                "i2c_bus": i2c_bus,
            }
        ],
        output="both",
    )

    nodes = [
        sensors,
    ]

    return LaunchDescription(arguments + nodes)
