from launch import LaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node


def generate_launch_description():
    # ARGUMENTS
    can_bus = LaunchConfiguration("can_bus", default="")
    # NODES
    rcb_driver = Node(
        package="perseus_can_if",
        executable="rcb_driver",
        parameters=[
            {"can_bus": can_bus},
        ],
        output="both",
    )

    return LaunchDescription(
        [
            rcb_driver,
        ]
    )
