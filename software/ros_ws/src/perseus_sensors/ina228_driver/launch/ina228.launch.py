from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "i2c_bus",
                default_value="/dev/i2c-7",
                description="I2C bus device file",
            ),
            DeclareLaunchArgument(
                "device_address",
                default_value="64",
                description="I2C device address (0x40 = 64 decimal)",
            ),
            DeclareLaunchArgument(
                "update_rate",
                default_value="10.0",
                description="Update rate in Hz",
            ),
            DeclareLaunchArgument(
                "shunt_resistance",
                default_value="0.015",
                description="Shunt resistor value in ohms",
            ),
            DeclareLaunchArgument(
                "max_current",
                default_value="10.0",
                description="Maximum expected current in amps",
            ),
            DeclareLaunchArgument(
                "required",
                default_value="false",
                description="Whether power monitor is required for system operation",
            ),
            Node(
                package="perseus_sensors",
                executable="ina228_node",
                name="ina228_node",
                parameters=[
                    {
                        "i2c_bus": LaunchConfiguration("i2c_bus"),
                        "device_address": LaunchConfiguration("device_address"),
                        "update_rate": LaunchConfiguration("update_rate"),
                        "shunt_resistance": LaunchConfiguration("shunt_resistance"),
                        "max_current": LaunchConfiguration("max_current"),
                        "required": LaunchConfiguration("required"),
                    }
                ],
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )
