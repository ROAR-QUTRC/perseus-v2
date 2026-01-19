from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perseus_autonomy_bridge",
                executable="nav2_waypoints_bridge",
                name="nav2_waypoints_bridge",
                output="screen",
                parameters=[
                    {
                        "waypoints_dir": "${HOME}/perseus-v2/software/web_ui/static"
                    }
                ],
            )
        ]
    )
