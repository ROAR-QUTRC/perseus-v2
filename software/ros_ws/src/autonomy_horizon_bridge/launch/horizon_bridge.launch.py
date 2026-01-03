from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="autonomy_horizon_bridge",
            executable="frontier_server",
            name="frontier_server",
            output="screen",
            parameters=[{
                "use_sim_time": True
            }, 
            # Adjust to correct path when installed:
            # This works best once installed; for dev, use absolute path if needed.
            ],
        ),
    ])
