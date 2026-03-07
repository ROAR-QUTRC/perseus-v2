from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Config file for the livox mid360 lidar on perseus
    # Note: this is a clone of the mid360.yaml on github/fast-lio with tested parameters
    config_file = os.path.join(
        get_package_share_directory("perseus_mapping"),
        "config",
        "livox_mid360.yaml",
    )

    # Launches the default fast-lio mapping launch file
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("fast_lio"),
                        "launch",
                        "mapping.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "config_file": config_file,
        }.items(),
    )

    return LaunchDescription(
        [
            fast_lio_launch,
        ]
    )
