"""
lidar_processing.launch.py - Livox Sensor with IMU Bias Correction and Topic Remapper

This launch file starts:
1. The Livox sensor driver (IMU and LiDAR)
2. The IMU Bias Estimator and Bias Remover composable nodes
3. The Pointcloud-to-Laserscan converter node

Usage:
    # Launch with default settings
    ros2 launch perseus_sensors lidar_processing.launch.py

    # Launch with custom IMU remap frequency
    ros2 launch perseus_sensors lidar_processing.launch.py imu_frequency:=20.0

Launch Arguments:
    scan_in        : Input LiDAR topic (default: /livox/lidar)
    scan_out       : Output LaserScan topic (default: /livox/scan)
    imu_frequency  : Target output frequency for remapped IMU in Hz (default: 50.0)
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
        description="Target output frequency for remapped IMU in Hz",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time if true",
    )

    scan_in = LaunchConfiguration("scan_in")
    scan_out = LaunchConfiguration("scan_out")
    use_sim_time = LaunchConfiguration("use_sim_time")

    imu_bias_container = ComposableNodeContainer(
        name="imu_bias_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        composable_node_descriptions=[
            ComposableNode(
                package="perseus_sensors",
                plugin="imu_processors::BiasEstimator",
                name="imu_bias_estimator",
                parameters=[
                    str(lidar_processing_config_file),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            ComposableNode(
                package="perseus_sensors",
                plugin="imu_processors::BiasRemover",
                name="imu_bias_remover",
                parameters=[
                    str(lidar_processing_config_file),
                    {"use_sim_time": use_sim_time},
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
        parameters=[
            pointcloud_to_laserscan_config_file,
            {"use_sim_time": use_sim_time},
        ],
        name="pointcloud_to_laserscan",
        output="screen",
    )

    # Create launch description and populate
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_publisher_name)
    ld.add_action(declare_subscriber_name)
    ld.add_action(imu_frequency_arg)
    ld.add_action(use_sim_time_arg)

    # Add nodes
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(imu_bias_container)

    return ld
