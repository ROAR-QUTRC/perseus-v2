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
    arguments = []

    # ENVIRONMENT VARIABLES
    env_vars = []

    # CONFIG + DATA FILES

    # XACRO FILES
    robot_description_xacro = PathJoinSubstitution(
        [FindPackageShare("perseus"), "urdf", "perseus.urdf.xacro"]
    )
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_xacro,
            " use_sim:=",
            use_sim_time,
        ]
    )

    # IMPORTED LAUNCH FILES
    launch_files = []

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

    nodes = [
        robot_state_publisher,
    ]

    # PROCESSES
    processes = []

    # EVENT HANDLERS
    handlers = []
    return LaunchDescription(
        env_vars + arguments + launch_files + nodes + processes + handlers
    )
