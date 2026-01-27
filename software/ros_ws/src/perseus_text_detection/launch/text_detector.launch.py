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
    arguments = []

    # ENVIRONMENT VARIABLES
    env_vars = []

    # CONFIG + DATA FILES

    # XACRO FILES

    # IMPORTED LAUNCH FILES
    launch_files = []

    # NODES
    text_detector = Node(
        package="perseus_text_detection",
        executable="text_detector",
        name="text detector",
        output="screen"
    )

    nodes = [
        text_detector,
    ]

    # PROCESSES
    processes = []

    # EVENT HANDLERS
    handlers = []
    return LaunchDescription(
        env_vars + arguments + launch_files + nodes + processes + handlers
    )
