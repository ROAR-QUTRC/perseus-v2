"""
Livox Sensor with Topic Remapper Launch File

This launch file starts:
1. The Livox sensor driver (IMU and LiDAR)
2. The topic remapper for downsampling IMU data to lower frequency

Usage:
    ros2 launch perseus_sensors livox_remap.launch.py
    
    With custom remapping frequency:
    ros2 launch perseus_sensors livox_remap.launch.py imu_frequency:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for Livox with topic remapper."""
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

    # Include the main livox launch file
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("perseus_sensors"), "launch", "livox.launch.py"]
            )
        ),
    )

    # Get launch configuration
    imu_frequency = LaunchConfiguration("imu_frequency")
    lidar_frequency = LaunchConfiguration("lidar_frequency")
    scan_in = LaunchConfiguration("scan_in")
    scan_out = LaunchConfiguration("scan_out")

    # Create topic remapper nodes for IMU and LiDAR
    imu_remapper_node = Node(
        package="perseus_sensors",
        executable="topic_remapper",
        name="imu_remapper",
        output="screen",
        parameters=[
            {"input_topic": "/livox/imu"},
            {"output_topic": "/livox/imu_remapped"},
            {"reduction_frequency": imu_frequency},
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

    # Add livox driver
    ld.add_action(livox_launch)

    # Add nodes
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(imu_remapper_node)

    return ld
