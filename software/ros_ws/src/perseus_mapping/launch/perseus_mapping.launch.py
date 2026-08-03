#!/usr/bin/env python3
"""Launch FAST-LIO with the Perseus MID-360 parameters.

Wraps fast_lio's own mapping.launch.py, feeding it config/livox_mid360.yaml. The only
processing done here is expanding ~ in map_file_path and creating that directory, which has
to happen in Python because fast_lio takes the path as a plain string, not a substitution.

Map saving is handled inside fast_lio itself, via the pcd_save block of that config:
periodically (save_interval_sec), on shutdown, and on demand via the /map_save service.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import tempfile


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the simulation clock. Only set true when something publishes /clock, "
        "otherwise nodes wait at startup and never run.",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz with fast_lio's own display config.",
    )

    config_path = os.path.join(
        get_package_share_directory("perseus_mapping"),
        "config",
    )

    # Load and resolve ~ in map_file_path before passing to fast_lio
    ros_config_file = os.path.join(config_path, "livox_mid360.yaml")
    with open(ros_config_file, "r") as f:
        ros_config = yaml.safe_load(f)

    params = ros_config.get("/**", {}).get("ros__parameters", {})
    if "map_file_path" in params:
        resolved_path = os.path.expanduser(params["map_file_path"])
        params["map_file_path"] = resolved_path
        # Ensure the maps directory exists
        os.makedirs(os.path.dirname(resolved_path), exist_ok=True)

    # Write resolved config to a temp file for fast_lio to consume
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(ros_config, tmp)
    tmp.close()
    resolved_config_path = os.path.dirname(tmp.name)
    resolved_config_file = os.path.basename(tmp.name)

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
            "config_path": resolved_config_path,
            "config_file": resolved_config_file,
            "rviz": rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription([declare_use_sim_time, declare_rviz, fast_lio_launch])
