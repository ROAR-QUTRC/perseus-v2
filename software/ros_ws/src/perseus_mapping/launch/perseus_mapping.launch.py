from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("perseus_mapping"),
        "config",
    )

    launch_config_file = os.path.join(config_path, "launch_config.yaml")
    with open(launch_config_file, "r") as f:
        launch_config = yaml.safe_load(f)

    use_sim_time = str(launch_config.get("use_sim_time", "false")).lower()
    rviz = str(launch_config.get("rviz", "false")).lower()
    sim = str(launch_config.get("sim", "false")).lower()

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fast_lio"),
                    "launch",
                    "mapping.launch.py",
                ]
            )
        ),
        launch_arguments={
            "config_path": config_path,
            "config_file": "livox_mid360.yaml",
            "rviz": rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    sim_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("perseus_mapping"),
                    "launch",
                    "sim_pointcloud_filter.launch.py",
                ]
            )
        ),
        condition=IfCondition(sim),
    )

    return LaunchDescription(
        [
            sim_filter_launch,
            fast_lio_launch,
        ]
    )
