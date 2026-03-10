from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

# Manual service call to save map:
# ros2 service call /map_save std_srvs/srv/Trigger {}


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("perseus_mapping"),
        "config",
    )

    launch_config_file = os.path.join(config_path, "launch_config.yaml")
    with open(launch_config_file, "r") as f:
        launch_config = yaml.safe_load(f)

    ros_config_file = os.path.join(config_path, "livox_mid360.yaml")
    with open(ros_config_file, "r") as f:
        ros_config = yaml.safe_load(f)

    map_file_path = ros_config["/**"]["ros__parameters"]["map_file_path"]

    map_saver_node = Node(
        package="perseus_mapping",
        executable="map_saver",
        name="map_saver",
        parameters=[
            {
                "use_sim_time": launch_config.get("use_sim_time", False),
                "save_interval_sec": launch_config.get("map_save_interval_sec", 30.0),
                "map_file_path": map_file_path,
            }
        ],
    )

    return LaunchDescription([map_saver_node])
