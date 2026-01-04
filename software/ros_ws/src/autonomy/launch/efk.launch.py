from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock",
        ),
    ]

    ekf_config_file = PathJoinSubstitution(
        [FindPackageShare("autonomy"), "config", "ekf_config.yaml"]
    )


    # Define EKF node before it's used in TimerAction
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_file, {"use_sim_time": use_sim_time}],
        # Explicit remapping to ensure proper topic connections
        remappings=[
            ("/odometry/filtered", "/odometry/filtered"),  # EKF output
        ],
    )

    launch_files = [
        ekf_node,  # EKF node
    ]

    return LaunchDescription(arguments + launch_files)
