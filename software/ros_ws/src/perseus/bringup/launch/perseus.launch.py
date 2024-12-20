from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_legacy_hardware = LaunchConfiguration("use_legacy_hardware")
    can_bus = LaunchConfiguration("can_bus")

    arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="False",
            description="Use mock hardware components which mirror commands to state interfaces",
        ),
        DeclareLaunchArgument(
            "use_legacy_hardware",
            default_value="True",
            description="Use legacy hardware (MCB) interfaces",
        ),
        DeclareLaunchArgument(
            "can_bus",
            default_value="can0",
            description="CAN bus to use for hardware communications",
        ),
    ]

    # CONFIG FILES
    controller_config = PathJoinSubstitution(
        [FindPackageShare("perseus"), "config", "perseus_controllers.yaml"]
    )

    # XACRO FILES
    # these all need to be run with the xacro command
    # to convert them to standard XML before they can be used
    robot_description_xacro = PathJoinSubstitution(
        [FindPackageShare("perseus"), "urdf", "perseus.urdf.xacro"]
    )
    # run xacro to generate the final output
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                # pass through all the arguments
                " ",
                robot_description_xacro,
                " ",
                "use_mock_hardware:=",
                use_mock_hardware,
                " ",
                "use_legacy_hardware:=",
                use_legacy_hardware,
                " ",
                "can_bus:=",
                can_bus,
            ]
        ),
        value_type=str,
    )

    # NODES
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="both",  # output to both screen and log file
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/perseus_base_controller/cmd_vel", "/cmd_vel"),
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
        ],
    )
    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "perseus_base_controller",
        ],
    )

    nodes = [
        robot_state_publisher,
        controller_manager,
        base_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    # EVENT HANDLERS
    handlers = []

    # actually return the launch description
    return LaunchDescription(arguments + nodes + handlers)
