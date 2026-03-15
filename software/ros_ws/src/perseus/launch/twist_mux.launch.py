from launch import LaunchDescription

from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    use_sim_time_param = {"use_sim_time": use_sim_time}

    arguments = []

    # CONFIG + DATA FILES
    mux_config = PathJoinSubstitution(
        [FindPackageShare("perseus"), "config", "twist_mux.yaml"]
    )

    # NODES
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        parameters=[use_sim_time_param, mux_config],
    )
    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        output="screen",
        parameters=[use_sim_time_param],
        remappings=[
            ("/cmd_vel_in", "/web_vel_unstamped"), # Foxglove extension publishes unstamped topic, remap to match expected input
            ("/cmd_vel_out", "/web_vel"),
        ],
    )

    nodes = [
        twist_mux,
        twist_stamper,
    ]

    return LaunchDescription(arguments + nodes)
