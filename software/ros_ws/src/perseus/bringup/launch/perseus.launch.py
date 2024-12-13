from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="False",
            description="Use mock hardware components which mirror commands to state interfaces",
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
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_xacro,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )

    # PARAMETERS FROM XACRO FILES
    robot_description = {"robot_description": robot_description_content}

    # NODES
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"lock_memory": False}],
        output="both",  # output to both screen and log file
        remappings=[],
        name="perseus_controller_manager",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    nodes = [controller_manager, robot_state_publisher]

    # OTHER

    # actually return the launch description
    return LaunchDescription(arguments + nodes)
