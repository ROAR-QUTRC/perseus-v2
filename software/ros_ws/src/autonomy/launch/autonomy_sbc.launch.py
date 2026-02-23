import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the SBC autonomy stack for the physical rover.

    This is the top-level launch file for the SBC (Single Board Computer).
    It runs the full autonomy stack (SLAM + Nav2 + EKF) and the Livox MID360
    LiDAR driver, plus foxglove_bridge for remote visualization.

    Does NOT include perseus.launch.py — RSP and hardware controllers run on
    the main rover PC, not the SBC.

    Does NOT launch the topic_republisher — that runs as a separate process
    with a different CYCLONEDDS_URI (bridge config with lo + network).
    """

    autonomy_dir = get_package_share_directory("autonomy")
    sensors_dir = get_package_share_directory("perseus_sensors")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock (false for physical rover)",
    )

    declare_foxglove_port = DeclareLaunchArgument(
        "foxglove_port",
        default_value="8765",
        description="WebSocket port for foxglove_bridge",
    )

    # Include the full autonomy stack (SLAM + Nav2 + EKF)
    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autonomy_dir, "launch", "autonomy.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
    )

    # Include Livox MID360 driver
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_dir, "launch", "livox.launch.py")
        ),
    )

    # Foxglove Bridge for WebSocket-based remote visualization
    # This bypasses DDS entirely — clients connect via TCP WebSocket,
    # so pointcloud data can be viewed without saturating the WiFi network.
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("foxglove_port"),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_foxglove_port)

    ld.add_action(autonomy_launch)
    ld.add_action(livox_launch)
    ld.add_action(foxglove_bridge_node)

    return ld
