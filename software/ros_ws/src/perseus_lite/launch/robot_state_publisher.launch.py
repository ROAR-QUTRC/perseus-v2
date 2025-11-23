from launch import LaunchDescription
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
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    serial_port = LaunchConfiguration("serial_port", default="")
    baud_rate = LaunchConfiguration("baud_rate", default="")

    # XACRO FILES
    robot_description_xacro = PathJoinSubstitution(
        [FindPackageShare("perseus_lite"), "urdf", "perseus_lite.urdf.xacro"]
    )
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_xacro,
            " use_sim:=",
            use_sim_time,
            " hardware_plugin:=",
            hardware_plugin,
            " serial_port:=",
            serial_port,
            " baud_rate:=",
            baud_rate,
        ]
    )

    # NODES
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": use_sim_time,
            }
        ],
        output="both",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
        ]
    )
