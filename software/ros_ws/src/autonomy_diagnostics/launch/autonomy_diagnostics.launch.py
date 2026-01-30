"""Launch file for autonomy diagnostics TUI."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for autonomy diagnostics."""
    return LaunchDescription(
        [
            Node(
                package="autonomy_diagnostics",
                executable="autonomy_tui",
                name="autonomy_diagnostics",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
