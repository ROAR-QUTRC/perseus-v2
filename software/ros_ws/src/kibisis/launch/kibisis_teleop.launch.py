from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_device = LaunchConfiguration("joy_device")

    arguments = [
        DeclareLaunchArgument(
            "joy_device",
            default_value="/dev/input/js0",
            description="Joystick device path",
        ),
    ]

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="both",
        emulate_tty=True,
        parameters=[{"device": joy_device, "deadzone": 0.05, "autorepeat_rate": 20.0}],
    )

    controller_node = Node(
        package="input_devices",
        executable="xbox_controller",
        name="xbox_controller",
        output="both",
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("kibisis"),
                    "config",
                    "xbox_controller_kibisis.yaml",
                ]
            )
        ],
        remappings=[
            ("/input_devices/cmd_vel", "/kibisis/diff_drive_base_controller/cmd_vel"),
        ],
    )

    return LaunchDescription(arguments + [joy_node, controller_node])
