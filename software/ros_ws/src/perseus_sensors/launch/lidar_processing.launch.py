"""
Livox Sensor Processing Launch File

This launch file starts:
1. A topic remapper to throttle the raw 200Hz /livox/imu down to a configurable rate
2. IMU bias estimation and removal pipeline (subscribes to throttled IMU)
3. Pointcloud to LaserScan converter

Usage:
    ros2 launch perseus_sensors lidar_processing.launch.py

    With custom IMU frequency:
    ros2 launch perseus_sensors lidar_processing.launch.py imu_frequency:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for Livox with topic remapper."""
    lidar_processing_config_file = os.path.join(
        get_package_share_directory("perseus_sensors"),
        "config",
        "lidar_processing_config.yaml",
    )
    pointcloud_to_laserscan_config_file = os.path.join(
        get_package_share_directory("perseus_sensors"),
        "config",
        "pcl_conv.yaml",
    )

    # Declare Arguments
    declare_publisher_name = DeclareLaunchArgument(
        "scan_out", default_value="/livox/scan", description="Publisher topic name"
    )
    declare_subscriber_name = DeclareLaunchArgument(
        "scan_in", default_value="/livox/lidar", description="Subscriber topic name"
    )

    # Declare launch arguments
    imu_frequency_arg = DeclareLaunchArgument(
        "imu_frequency",
        default_value="50.0",
        description="Target output frequency for throttled IMU in Hz",
    )

    scan_in = LaunchConfiguration("scan_in")
    scan_out = LaunchConfiguration("scan_out")
    imu_frequency = LaunchConfiguration("imu_frequency")

    # Throttle the raw 200Hz /livox/imu down to imu_frequency
    imu_throttle_node = Node(
        package="perseus_sensors",
        executable="topic_remapper",
        name="imu_throttle",
        output="screen",
        parameters=[
            {
                "input_topic": "/livox/imu",
                "output_topic": "/livox/imu/throttled",
                "reduction_frequency": imu_frequency,
            }
        ],
    )

    # Bias estimation and removal pipeline — subscribes to throttled IMU
    imu_bias_container = ComposableNodeContainer(
        name="imu_bias_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="perseus_sensors",
                plugin="imu_processors::BiasEstimator",
                name="imu_bias_estimator",
                parameters=[
                    str(lidar_processing_config_file),
                    {"imu_in_topic": "/livox/imu/throttled"},
                ],
            ),
            ComposableNode(
                package="perseus_sensors",
                plugin="imu_processors::BiasRemover",
                name="imu_bias_remover",
                parameters=[
                    str(lidar_processing_config_file),
                    {"imu_in_topic": "/livox/imu/throttled"},
                ],
            ),
        ],
    )

    # Pointcloud to Laserscan converter node
    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[
            ("cloud_in", scan_in),
            ("scan_out", scan_out),
        ],
        parameters=[pointcloud_to_laserscan_config_file],
        name="pointcloud_to_laserscan",
        output="screen",
    )

    # Create launch description and populate
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_publisher_name)
    ld.add_action(declare_subscriber_name)
    ld.add_action(imu_frequency_arg)

    # Add nodes
    ld.add_action(imu_throttle_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(imu_bias_container)

    return ld
