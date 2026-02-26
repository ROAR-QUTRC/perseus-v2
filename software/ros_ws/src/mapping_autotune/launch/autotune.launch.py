import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    autonomy_share = get_package_share_directory("autonomy")

    db_path = LaunchConfiguration("db_path")
    max_runs = LaunchConfiguration("max_runs")
    session_name = LaunchConfiguration("session_name")
    maneuver_pattern = LaunchConfiguration("maneuver_pattern")
    settling_time = LaunchConfiguration("settling_time")
    auto_calibrate_imu = LaunchConfiguration("auto_calibrate_imu")
    interactive = LaunchConfiguration("interactive")
    linear_speed = LaunchConfiguration("linear_speed")
    rotation_speed = LaunchConfiguration("rotation_speed")

    arguments = [
        DeclareLaunchArgument(
            "db_path",
            default_value="~/.local/share/mapping_autotune/autotune.db",
            description="Path to the SQLite database file",
        ),
        DeclareLaunchArgument(
            "max_runs",
            default_value="10",
            description="Maximum number of autotune runs",
        ),
        DeclareLaunchArgument(
            "session_name",
            default_value="",
            description="Name for this autotune session (auto-generated if empty)",
        ),
        DeclareLaunchArgument(
            "maneuver_pattern",
            default_value="box_return",
            description="Maneuver pattern: box_return or corridor",
        ),
        DeclareLaunchArgument(
            "settling_time",
            default_value="3.0",
            description="Seconds to wait after SLAM restart before maneuver",
        ),
        DeclareLaunchArgument(
            "auto_calibrate_imu",
            default_value="true",
            description="Auto-calibrate IMU gyro bias on startup",
        ),
        DeclareLaunchArgument(
            "interactive",
            default_value="true",
            description="Show setup TUI before starting autotune",
        ),
        DeclareLaunchArgument(
            "linear_speed",
            default_value="0.2",
            description="Default linear speed for maneuvers (m/s)",
        ),
        DeclareLaunchArgument(
            "rotation_speed",
            default_value="0.5",
            description="Default rotation speed for maneuvers (rad/s)",
        ),
    ]

    # IMU filter node — relays /imu/data to /imu/data_filtered with bias/deadband
    imu_filter = Node(
        package="mapping_autotune",
        executable="imu_filter_node",
        name="imu_filter_node",
        output="screen",
        parameters=[
            {
                "auto_calibrate": auto_calibrate_imu,
                "calibration_duration": 30.0,
                "gyro_deadband": 0.01,
            }
        ],
    )

    # Main autotune orchestrator node
    autotune = Node(
        package="mapping_autotune",
        executable="autotune_node",
        name="autotune_node",
        output="screen",
        parameters=[
            {
                "db_path": db_path,
                "max_runs": max_runs,
                "session_name": session_name,
                "maneuver_pattern": maneuver_pattern,
                "settling_time": settling_time,
                "interactive": interactive,
                "linear_speed": linear_speed,
                "rotation_speed": rotation_speed,
                "slam_config_path": os.path.join(
                    autonomy_share, "config", "slam_toolbox_params_perseus_lite.yaml"
                ),
                "ekf_config_path": os.path.join(
                    autonomy_share, "config", "ekf_params_perseus_lite.yaml"
                ),
            }
        ],
    )

    return LaunchDescription(arguments + [imu_filter, autotune])
