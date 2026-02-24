from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("as7343_driver")
    default_config = os.path.join(pkg_dir, "config", "as7343_config.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to AS7343 configuration YAML file",
            ),
            DeclareLaunchArgument(
                "i2c_bus",
                default_value="/dev/i2c-1",
                description="I2C bus device file",
            ),
            DeclareLaunchArgument(
                "required",
                default_value="false",
                description="Whether sensor is required for system operation",
            ),
            Node(
                package="as7343_driver",
                executable="as7343_node",
                name="as7343_node",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "i2c_bus": LaunchConfiguration("i2c_bus"),
                        "required": LaunchConfiguration("required"),
                    },
                ],
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )
