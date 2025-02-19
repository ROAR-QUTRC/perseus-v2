from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    turn_speed = LaunchConfiguration("turn_speed")
    drive_speed = LaunchConfiguration("drive_speed")
    high_speed_multiplier = LaunchConfiguration("high_speed_multiplier")

    arguments = [
        DeclareLaunchArgument(
            "turn_speed",
            default_value="0.5",
            description="Turn speed in rad/s",
        ),
        DeclareLaunchArgument(
            "drive_speed",
            default_value="0.25",
            description="Drive speed in m/s",
        ),
        DeclareLaunchArgument(
            "high_speed_multiplier",
            default_value="2.0",
            description="High speed multiplier for drive and turn speeds",
        ),
    ]

    # NODES
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="both",
        emulate_tty=True,
    )
    controller_node = Node(
        package="input_devices",
        executable="xbox_controller",
        name="xbox_controller",
        output="both",
        emulate_tty=True,
        parameters=[
            {
                "use_stamped_msg": True,
                "translation_scale": drive_speed,
                "rotation_scale": turn_speed,
                "high_high_speed_multiplier": high_speed_multiplier,
            }
        ],
        remappings=[("~/cmd_vel", "/cmd_vel")],
    )
    nodes = [joy_node, controller_node]

    return LaunchDescription(arguments + nodes)
