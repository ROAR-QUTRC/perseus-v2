from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # ARGUMENTS
    can_bus = LaunchConfiguration("can_bus")

    arguments = [
        DeclareLaunchArgument(
            "can_bus",
            default_value="can0",
            description="The CAN network interface to use",
        ),
    ]

    # NODES
    bucket_driver = Node(
        package="perseus_payloads",
        executable="bucket_driver",
        parameters=[can_bus],
        output="both",
    )
    bucket_controller = Node(
        package="perseus_payloads",
        executable="bucket_controller",
        parameters=[],
        output="both",
    )

    nodes = [bucket_driver, bucket_controller]

    return LaunchDescription(arguments + nodes)
