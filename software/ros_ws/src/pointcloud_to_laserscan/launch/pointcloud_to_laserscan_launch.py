from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("pointcloud_to_laserscan"),
        "config",
        "pointcloud_to_laserscan.yaml",
    )

    # Declare Arguments
    declare_publisher_name = DeclareLaunchArgument(
        "scan_out", default_value="/livox/scan", description="Publisher topic name"
    )
    declare_subscriber_name = DeclareLaunchArgument(
        "scan_in", default_value="/livox/lidar", description="Subscriber topic name"
    )

    return LaunchDescription(
        [
            declare_subscriber_name,
            declare_publisher_name,
            DeclareLaunchArgument(
                name="scanner",
                default_value="scanner",
                description="converts lidar pointcloud to scan",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                remappings=[
                    ("cloud_in", LaunchConfiguration("scan_in")),
                    ("scan_out", LaunchConfiguration("scan_out")),
                ],
                parameters=[config_file],
                name="pointcloud_to_laserscan",
            ),
        ]
    )
