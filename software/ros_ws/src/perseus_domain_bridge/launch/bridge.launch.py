from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import os


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value="",
        description=(
            "Absolute path to the throttled bridge YAML config. "
            "Leave empty to use the package default (config/bridge_config.yaml)."
        ),
    )

    def launch_setup(context, *args, **kwargs):

        config_dds = LaunchConfiguration("config").perform(context)
        config_rover_to_laptop = LaunchConfiguration("config").perform(context)
        config_laptop_to_rover = LaunchConfiguration("config").perform(context)

        if not config_dds:
            from ament_index_python.packages import get_package_share_directory

            pkg_share = get_package_share_directory("perseus_domain_bridge")
            config_dds = os.path.join(pkg_share, "config", "rover_bridge.xml")
            config_rover_to_laptop = os.path.join(
                pkg_share, "config", "rover_to_laptop.yaml"
            )
            config_laptop_to_rover = os.path.join(
                pkg_share, "config", "laptop_to_rover.yaml"
            )

        return [
            # Use bridge domain XML for the bridge process
            SetEnvironmentVariable("CYCLONEDDS_URI", f"file://{config_dds}"),
            # Rover → Laptop topics
            Node(
                package="domain_bridge",
                executable="domain_bridge",
                name="rover_to_laptop_bridge",
                arguments=[config_rover_to_laptop],
            ),
            # Laptop → Rover topics
            Node(
                package="domain_bridge",
                executable="domain_bridge",
                name="laptop_to_rover_bridge",
                arguments=[config_laptop_to_rover],
            ),
        ]

    return LaunchDescription(
        [
            config_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
