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
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    hardware_plugin = LaunchConfiguration("hardware_plugin")

    robot_description_xacro = PathJoinSubstitution(
        [FindPackageShare("kibisis"), "urdf", "kibisis.urdf.xacro"]
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
        ]
    )

    # Namespaced under /kibisis — robot_description published on
    # /kibisis/robot_description, joint_states on /kibisis/joint_states
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="kibisis",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str
                ),
                "use_sim_time": use_sim_time,
                # Publish tf under kibisis/ prefix to avoid clashing with perseus
                "frame_prefix": "kibisis/",
            }
        ],
        output="both",
    )

    return LaunchDescription([robot_state_publisher])
