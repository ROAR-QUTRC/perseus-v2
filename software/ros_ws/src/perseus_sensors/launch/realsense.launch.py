"""
RealSense Camera Launch File (+ Image Compression)

This launch file loads RealSense settings from a YAML file, forwards the
camera-related options to `realsense2_camera/rs_launch.py`, and optionally
starts image_transport republishers for compressed image topics.

Usage:
    ros2 launch perseus_sensors rs_launch.py

    ros2 launch perseus_sensors rs_launch.py \
        config_file:=/absolute/path/to/realsense_launch.yaml
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_file = str(
        Path(get_package_share_directory("perseus_sensors"))
        / "config"
        / "realsense_launch.yaml"
    )

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={"config_file": LaunchConfiguration("config_file")}.items(),
    )

    rgb_compress = Node(
        package="image_transport",
        executable="republish",
        name="rgb_compress_republish",
        output="screen",
        arguments=["raw", "compressed"],
        remappings=[
            ("in", LaunchConfiguration("rgb_in")),
            ("out", LaunchConfiguration("rgb_out")),
        ],
        condition=IfCondition(LaunchConfiguration("enable_image_compression")),
    )

    depth_compress = Node(
        package="image_transport",
        executable="republish",
        name="depth_compress_republish",
        output="screen",
        arguments=["raw", "compressedDepth"],
        remappings=[
            ("in", LaunchConfiguration("depth_in")),
            ("out", LaunchConfiguration("depth_out")),
        ],
        condition=IfCondition(LaunchConfiguration("enable_image_compression")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="YAML config file containing RealSense launch settings",
            ),
            DeclareLaunchArgument(
                "enable_image_compression",
                default_value="true",
                description="Start image_transport republishers for compressed topics",
            ),
            DeclareLaunchArgument(
                "rgb_in",
                default_value="/camera/camera/color/image_raw",
                description="Input RGB image topic to compress",
            ),
            DeclareLaunchArgument(
                "rgb_out",
                default_value="/camera/camera/color/image_compressed",
                description="Output RGB compressed image topic",
            ),
            DeclareLaunchArgument(
                "depth_in",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
                description="Input depth image topic to compress",
            ),
            DeclareLaunchArgument(
                "depth_out",
                default_value="/camera/camera/depth/image_compressedDepth",
                description="Output depth compressed image topic",
            ),
            rs_launch,
            rgb_compress,
            depth_compress,
        ]
    )
