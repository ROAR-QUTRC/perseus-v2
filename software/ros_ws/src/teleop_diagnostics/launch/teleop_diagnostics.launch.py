from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the teleop diagnostics TUI."""
    teleop_diagnostics = Node(
        package="teleop_diagnostics",
        executable="teleop_tui",
        name="teleop_diagnostics",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([teleop_diagnostics])
