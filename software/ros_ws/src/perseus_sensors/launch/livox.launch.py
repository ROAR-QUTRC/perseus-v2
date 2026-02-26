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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Config Handling
import json
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import subprocess


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
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "interface",
                default_value="eth1",
                description="Network interface to get host IP from",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
