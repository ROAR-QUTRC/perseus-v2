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
    config = LaunchConfiguration("config")
    debug = LaunchConfiguration("debug")

    arguments = [
        DeclareLaunchArgument(
            "type",
            default_value="xbox",
            description="Controller type: Either 'xbox' or '8bitdo'",
        ),
        DeclareLaunchArgument(
            "wireless",
            default_value="true",
            description="Is the controller connected wirelessly or with a cable?",
        ),
        DeclareLaunchArgument(
            "config",
            default_value="",
            description="Path to config file, overrides 'wireless' and 'type'",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="true",
            description="Boolean value for debugging the controller"
        )
    ]

    # CONFIG + DATA FILES
    is_xbox = EqualsSubstitution(controller_type, "xbox")
    xbox_controller_config = [
        "xbox_controller",
        IfElseSubstitution(is_wireless, "_wireless.yaml", "_wired.yaml"),
    ]
    eightbitdo_controller_config = ["8bitdo_controller.yaml"]
    controller_config_name = IfElseSubstitution(
        is_xbox, xbox_controller_config, eightbitdo_controller_config
    )
    preferred_config_path = PathJoinSubstitution(
        [FindPackageShare("perseus_input_config"), "config", controller_config_name]
    )
    config_path = IfElseSubstitution(
        EqualsSubstitution(config, ""), preferred_config_path, config
    )
    debug_arguments = IfElseSubstitution(
            EqualsSubstitution(debug, "true"),
            ["--ros-args", "--log-level", "generic_controller:=debug"],
            [],
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
        parameters=[config_path],
        remappings=[],
    )
    nodes = [joy_node, controller_node]

    return LaunchDescription(arguments + nodes)
