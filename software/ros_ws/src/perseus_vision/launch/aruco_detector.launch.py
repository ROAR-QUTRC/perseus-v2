from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('perseus_vision'),
        'config',
        'aruco_detector_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='perseus_vision',
            executable='detector_node',  # Update this to match your CMake target name
            name='aruco_detector',
            output='screen',
            parameters=[config_file]
        )
    ])
