import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    is_rover = LaunchConfiguration("rover").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    domain_id = int(LaunchConfiguration("domain_id").perform(context))

    config_filename = (
        "autonomous_to_main.yaml" if is_rover else "main_to_autonomous.yaml"
    )

    config_file = os.path.join(
        get_package_share_directory(
            "your_package_name"
        ),  # <-- replace with your package name
        "config",
        config_filename,
    )

    # Load the config and override from_domain with the launch arg
    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    config["from_domain"] = domain_id

    # Write to a temp file for domain_bridge to consume
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="domain_bridge_"
    )
    yaml.dump(config, tmp)
    tmp.close()

    domain_bridge_node = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name="domain_bridge",
        output="screen",
        arguments=[tmp.name],
    )

    return [domain_bridge_node]


def generate_launch_description():
    rover_arg = DeclareLaunchArgument(
        "rover",
        default_value="false",
        description="Set to true when launching on the rover, false for the main station",
    )

    domain_id_arg = DeclareLaunchArgument(
        "domain_id",
        default_value="0",  # rover default; pass 2 on main station
        description="ROS_DOMAIN_ID of this device (from_domain). Rover=0, Main=2",
    )

    return LaunchDescription(
        [
            rover_arg,
            domain_id_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
