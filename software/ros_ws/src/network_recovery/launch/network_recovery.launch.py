import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("network_recovery")
    params_file = os.path.join(pkg_dir, "config", "network_recovery_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    network_recovery_node = Node(
        package="network_recovery",
        executable="network_recovery_node",
        name="network_recovery",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            network_recovery_node,
        ]
    )
