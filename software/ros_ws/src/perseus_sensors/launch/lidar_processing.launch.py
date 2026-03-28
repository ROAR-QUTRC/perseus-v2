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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import json
import os
import subprocess
import tempfile
from pathlib import Path


def create_livox_config(default_config_path: str, interface: str):
    """
    Creates a temporary file with correct host IP that the driver uses.
    """

    config_path = Path(default_config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    def get_host_ip(interface: str):
        result = subprocess.run(
            ["ip", "-4", "addr", "show", interface],
            capture_output=True,
            text=True,
            check=True,
        )

        for line in result.stdout.splitlines():
            line = line.strip()
            if line.startswith("inet "):
                return line.split()[1].split("/")[0]

        raise RuntimeError(f"No IPv4 address found on {interface}")

    host_ip = get_host_ip(interface)

    # Load default config
    with open(config_path, "r") as f:
        config = json.load(f)

    # Insert HOST IP into config
    if "MID360" in config and "host_net_info" in config["MID360"]:
        for key in config["MID360"]["host_net_info"]:
            if key.endswith("_ip"):
                config["MID360"]["host_net_info"][key] = host_ip

    # Create temp config file
    with tempfile.NamedTemporaryFile(
        mode="wt",
        dir="/tmp",
        delete=False,
        prefix="livox.",
    ) as temp:
        json.dump(config, temp, indent=2)

        print("=========================")
        print("|   TEMP FILE CREATED   |")
        print("=========================")
        print(f"Located: {temp.name}")
        print(f"Interface: {interface}")
        print(f"Host IP: {host_ip}")
        print("=========================")

        return temp.name


def launch_setup(context, *args, **kwargs):
    """
    Runtime launch setup (required to resolve LaunchConfiguration)
    """

    interface = LaunchConfiguration("interface").perform(context)
    use_sim_time_val = LaunchConfiguration("use_sim_time").perform(context)

    # Only launch Livox if not using simulated time
    if use_sim_time_val.lower() == "true":
        return []

    config_path = (
        Path(get_package_share_directory("perseus_sensors"))
        / "config"
        / "livox_config.json"
    )

    livox_path = create_livox_config(str(config_path), interface)

    return [
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            parameters=[
                {
                    "xfer_format": 0,
                    "multi_topic": 0,
                    "data_src": 0,
                    "publish_freq": 10.0,
                    "output_data_type": 0,
                    "frame_id": "livox_frame",
                    "user_config_path": livox_path,
                }
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    """Generate launch description for Livox with topic remapper."""
    lidar_processing_config_file = os.path.join(
        get_package_share_directory("perseus_sensors"),
        "config",
        "lidar_processing_config.yaml",
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

    # Create launch description and populate
    ld = LaunchDescription()

    # Declare interface argument for Livox
    declare_interface_arg = DeclareLaunchArgument(
        "interface",
        default_value="eth1",
        description="Network interface to get host IP from",
    )

    # Add arguments
    ld.add_action(declare_interface_arg)
    ld.add_action(imu_frequency_arg)
    ld.add_action(use_sim_time_arg)

    ld.add_action(imu_bias_container)

    # Add Livox driver launch
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
