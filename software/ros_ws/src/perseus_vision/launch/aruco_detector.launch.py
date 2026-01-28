from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    perseus_vision_dir = get_package_share_directory("perseus_vision")
    config_dir = os.path.join(perseus_vision_dir, "config")
    config_file = os.path.join(config_dir, "perseus_vision.yaml")

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )

    # Create the aruco_detector node
    aruco_detector_node = Node(
        package="perseus_vision",
        executable="aruco_detector_node",
        name="aruco_detector",
        parameters=[config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            aruco_detector_node,
        ]
    )
