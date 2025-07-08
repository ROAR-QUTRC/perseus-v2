# ============================================================= #
#                                  ____                         #
#                                 / . .\                        #
#                                 \  ---<                       #
#                                  \  /                         #
#                        __________/ /                          #
#                     -=:___________/                           #
# ------------------------------------------------------------- #
#                         Livox runner                          #
#                        [ Ask Dan :) ]                         #  
# ------------------------------------------------------------- #
#                                                               #
# This launch file launches the livox driver.                   #
# It publishes a 3D pointcloud in the livox_frame.              #
# It uses the config file found in config/livox_config.json.    #
#                                                               #
# It dynamically changes the IP in the config if it changes.    #
# The Livox IP should be 192.168.1.21, if not network is weird. #
#                                                               #
# ============================================================= #

# ROS Things
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Config Handling
import json
import shutil
import socket
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def update_livox_config(config_file_path: str):
    """
    Update the host IP in the livox_config.json file based on the current IP address of the host.
    """
    config_path = Path(config_file_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    # Get the current IP address of the host (non-loopback)
    def get_host_ip():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # doesn't need to be reachable
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
        finally:
            s.close()
        return ip

    host_ip = get_host_ip()

    # Backup the original file
    backup_path = config_path.with_suffix(".json.bak")
    shutil.copy(config_path, backup_path)

    # Load, modify and save the JSON
    with open(config_path, "r") as f:
        config = json.load(f)

    if "MID360" in config and "host_net_info" in config["MID360"]:
        for key, value in config["MID360"]["host_net_info"].items():
            if key.endswith("_ip"):
                config["MID360"]["host_net_info"][key] = host_ip

    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

    print(f"Updated host IP to {host_ip} in {config_path}, backup saved to {backup_path}")


def generate_launch_description():
    # Real file path
    config_path = Path(
        get_package_share_directory("perseus_sensors")
    ) / "config" / "livox_config.json"

    update_livox_config(str(config_path))

    # Construct the path to the config file
    config_path = PathJoinSubstitution(
        [FindPackageShare("perseus_sensors"), "config", "livox_config.json"]
    )

    return LaunchDescription(
        [
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
                        "user_config_path": config_path,  # Use the resolved path here
                    }
                ],
                output="screen",
            ),
        ]
    )
