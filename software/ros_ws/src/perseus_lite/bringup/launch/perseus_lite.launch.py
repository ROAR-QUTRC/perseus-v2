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
    hardware_type = LaunchConfiguration("hardware_type")
    serial_port = LaunchConfiguration("serial_port")

    arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="False",
            description="Use mock hardware components which mirror commands to state interfaces",
        ),
        DeclareLaunchArgument(
            "hardware_type",
            default_value="st3215",
            description="Hardware type (st3215/mock)",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port for ST3215 servos",
        ),
    ]

    # CONFIG FILES
    controller_config = PathJoinSubstitution(
        [FindPackageShare("perseus_lite"), "config", "perseus_lite_controllers.yaml"]
    )

    # XACRO FILES
    robot_description_xacro = PathJoinSubstitution(
        [
            FindPackageShare("perseus_lite_description"),
            "urdf",
            "perseus_lite.urdf.xacro",
        ]
    )

    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                robot_description_xacro,
                " ",
                # Change this to match perseus_lite_hardware plugin name
                "hardware_plugin:=perseus_lite_hardware/ST3215SystemHardware",
                " ",
                "use_mock_hardware:=",
                use_mock_hardware,
                " ",
                "hardware_type:=",
                hardware_type,
                " ",
                "serial_port:=",
                serial_port,
            ]
        ),
        value_type=str,
    )

    # NODES
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diff_drive_base_controller/cmd_vel", "/cmd_vel"),
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
        arguments=["joint_state_broadcaster"],
    )

    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"],
    )

    nodes = [
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        base_controller_spawner,
    ]

    return LaunchDescription(arguments + nodes)
