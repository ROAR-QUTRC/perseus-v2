from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():

    # Correct path to YAML config
    config_file = os.path.join(
        get_package_share_directory("perseus_sensors"),
        "config",
        "lidar_segmentation.yaml",
        get_package_share_directory("perseus_sensors"),
        "config",
        "lidar_segmentation.yaml",
    )

    lidar_node = Node(
        package="perseus_sensors",
        executable="lidar_segmentation",
        name="lidar_segmentation_node",  # <-- must match YAML
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription(
        [
            lidar_node,
        ]
    )
