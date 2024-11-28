from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="input_devices",
                executable="xbox_controller",
                name="xbox_controller",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
