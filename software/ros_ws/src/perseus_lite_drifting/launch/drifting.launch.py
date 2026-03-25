import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("perseus_lite_drifting")
    config = os.path.join(pkg_share, "config", "drifting_params.yaml")

    return LaunchDescription(
        [
            Node(
                package="perseus_lite_drifting",
                executable="drift_detector_node",
                name="drift_detector_node",
                parameters=[config],
                output="screen",
            ),
        ]
    )
