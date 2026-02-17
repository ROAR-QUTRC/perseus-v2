from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    IfElseSubstitution,
    EqualsSubstitution,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    controller_type = LaunchConfiguration("type")
    is_wireless = LaunchConfiguration("wireless")
    dual_stick = LaunchConfiguration("dual_stick")
    config = LaunchConfiguration("config")
    timeout_enable = LaunchConfiguration("timeout_enable")
    debug = LaunchConfiguration("debug")

    arguments = [
        DeclareLaunchArgument(
            "type",
            default_value="xbox",
            description="Controller type: Either 'xbox', '8bitdo', or 'logitech'",
        ),
        DeclareLaunchArgument(
            "wireless",
            default_value="true",
            description="Is the controller connected wirelessly or with a cable?",
        ),
        DeclareLaunchArgument(
            "dual_stick",
            default_value="false",
            description="Use dual-stick driving (left=forward/back, right=turning) for Xbox controllers",
        ),
        DeclareLaunchArgument(
            "config",
            default_value="",
            description="Path to config file, overrides 'wireless' and 'type'",
        ),
        DeclareLaunchArgument(
            "timeout_enable",
            default_value="true",
            description="Should the controller timeout run (disable this during autonomy testing)",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="true",
            description="Boolean value for debugging the controller",
        ),
    ]

    # CONFIG + DATA FILES
    is_xbox = EqualsSubstitution(controller_type, "xbox")
    is_logitech = EqualsSubstitution(controller_type, "logitech")
    is_dual_stick = EqualsSubstitution(dual_stick, "true")
    controller_configs = {
        "xbox": [
            "xbox_controller",
            IfElseSubstitution(
                is_wireless,
                IfElseSubstitution(
                    is_dual_stick, "_wireless_dual_stick.yaml", "_wireless.yaml"
                ),
                IfElseSubstitution(
                    is_dual_stick, "_wired_dual_stick.yaml", "_wired.yaml"
                ),
            ),
        ],
        "logitech": ["logitech_controller.yaml"],
        "8bitdo": ["8bitdo_controller.yaml"],
    }
    controller_config_name = IfElseSubstitution(
        is_xbox,
        controller_configs["xbox"],
        IfElseSubstitution(
            is_logitech, controller_configs["logitech"], controller_configs["8bitdo"]
        ),
    )
    preferred_config_path = PathJoinSubstitution(
        [FindPackageShare("perseus_input_config"), "config", controller_config_name]
    )
    config_path = IfElseSubstitution(
        EqualsSubstitution(config, ""), preferred_config_path, config
    )
    debug_arg = IfElseSubstitution(
        EqualsSubstitution(debug, "true"),
        "generic_controller:=DEBUG",
        "generic_controller:=INFO",
    )

    # NODES
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="both",
        emulate_tty=True,
    )
    controller_node = Node(
        package="perseus_input",
        executable="generic_controller",
        name="generic_controller",
        output="both",
        emulate_tty=True,
        parameters=[config_path, timeout_enable],
        remappings=[],
        ros_arguments=["--log-level", debug_arg],
    )
    nodes = [joy_node, controller_node]

    return LaunchDescription(arguments + nodes)
