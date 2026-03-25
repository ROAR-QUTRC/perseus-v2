import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("crater_exit")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            Node(
                package="crater_exit",
                executable="crater_exit_node",
                name="crater_exit_node",
                output="screen",
                parameters=[
                    os.path.join(pkg_dir, "config", "crater_exit_params.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
