from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    serial_port = LaunchConfiguration("serial_port", default="")
    baud_rate = LaunchConfiguration("baud_rate", default="")
    servo_max_rpm = LaunchConfiguration("servo_max_rpm", default="62.0")

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
            " servo_max_rpm:=",
            servo_max_rpm,
        ]
    )

    # NODES
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str
                ),
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
