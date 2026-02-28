import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("space_resources_transceiver")
    params_file = os.path.join(pkg_dir, "config", "transceiver_params.yaml")

    transceiver_node = Node(
        package="space_resources_transceiver",
        executable="transceiver_node",
        name="transceiver_node",
        namespace="space_resources/end_effector",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([transceiver_node])
