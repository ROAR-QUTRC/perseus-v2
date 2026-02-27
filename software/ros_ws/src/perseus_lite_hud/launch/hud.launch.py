import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("perseus_lite_hud")
    params_file = os.path.join(pkg_dir, "config", "hud_params.yaml")

    hud_node = Node(
        package="perseus_lite_hud",
        executable="hud_overlay_node",
        name="hud_overlay_node",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([hud_node])
