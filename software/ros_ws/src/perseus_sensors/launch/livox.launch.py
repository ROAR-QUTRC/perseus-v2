# ============================================================= #
#                                  ____                         #
#                                 / . .\                        #
#                                 \  ---<                       #
#                                  \  /                         #
#                        __________/ /                          #
#                     -=:___________/                           #
# ------------------------------------------------------------- #
#                         Livox runner                          #
#               [ Ask Dan :) if something breaks ]              #
# ------------------------------------------------------------- #
#                                                               #
# This launch file launches the livox driver.                   #
# It publishes a 3D pointcloud in the livox_frame.              #
# It uses the config file found in config/livox_config.json.    #
#                                                               #
# It creates a tmp file with the host IP.                       #
# The Livox IP should be 192.168.1.21, if not network is weird. #
#                                                               #
# ============================================================= #

# ROS Things
from launch import LaunchDescription
from launch_ros.actions import Node

# Config Handling
import json
import socket
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def create_livox_config(default_config_path: str):
    """
    Creates a temporary file with correct host IP that the driver uses.
    """
    # Get path
    config_path = Path(default_config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    # Best stdlib way of getting IP
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

    # Loading default LIVOX config
    with open(config_path, "r") as f:
        config = json.load(f)

    # Insert HOST IP to config
    if "MID360" in config and "host_net_info" in config["MID360"]:
        for key, _ in config["MID360"]["host_net_info"].items():
            if key.endswith("_ip"):
                config["MID360"]["host_net_info"][key] = host_ip

    # Create temp file (starts with livox.******)
    with tempfile.NamedTemporaryFile(
        mode="wt",
        dir="/tmp",
        delete=False,
        prefix="livox.",
    ) as temp:
        # Write updated config to temp
        json.dump(config, temp, indent=2)

        print("=========================")
        print("|   TEMP FILE CREATED   |")
        print("=========================")
        print(f"Located: {temp.name}")
        print(f"Host IP: {host_ip}")
        print("=========================")
        return temp.name


def generate_launch_description():
    # Real file path
    config_path = (
        Path(get_package_share_directory("perseus_sensors"))
        / "config"
        / "livox_config.json"
    )

    # Temp file path
    livox_path = create_livox_config(str(config_path))

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
                        "user_config_path": livox_path,
                    }
                ],
                output="screen",
            ),
        ]
    )
