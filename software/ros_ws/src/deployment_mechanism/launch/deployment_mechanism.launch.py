from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="deployment_mechanism",
                executable="deployment_mechanism",
                name="deployment_mechanism",
                output="screen",
                parameters=[{"cmd_timeout_s": 0.5}],
            )
        ]
    )
