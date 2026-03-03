from launch import LaunchDescription
from launch_ros.actions import Node


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
        name="text_detector",
        output="screen",
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
