#!/usr/bin/env python3
"""Launch the localisation stack: FAST-LIO odometry fused by the EKF.

Two pieces, and the split matters:

  1. fast_lio provides LiDAR-inertial odometry from config/livox_mid360.yaml. It publishes
     /Odometry as odom -> base_link and broadcasts no TF of its own.
  2. ekf.launch.py runs robot_localization from config/ekf_config.yaml, fusing that pose
     with IMU angular velocity and owning the odom -> base_link transform.

Start order does not matter. fast_lio withholds /Odometry until lid_frame -> base_frame
resolves in TF, so until robot_state_publisher is up the EKF runs on the IMU alone.

Map saving is handled inside fast_lio, via the pcd_save block of livox_mid360.yaml:
periodically, on shutdown, and on demand via the /map_save service.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    ekf_params_file = LaunchConfiguration("ekf_params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the simulation clock. Only set true when something publishes "
        "/clock, otherwise the EKF waits at startup and never runs.",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz with fast_lio's own display config.",
    )

    declare_ekf_params_file = DeclareLaunchArgument(
        "ekf_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("autonomy"), "config", "ekf_config.yaml"]
        ),
        description="Parameters file for the robot_localization EKF.",
    )

    # The FAST-LIO config is read here rather than passed as a substitution because ~ in
    # map_file_path has to be expanded, and the directory created, before the node starts:
    # fast_lio takes that path as a plain string. The expanded copy goes to a temp file,
    # which is what fast_lio's own launch is pointed at.
    fast_lio_params_file = os.path.join(
        get_package_share_directory("autonomy"), "config", "livox_mid360.yaml"
    )
    with open(fast_lio_params_file, "r") as f:
        fast_lio_params = yaml.safe_load(f)

    params = fast_lio_params.get("/**", {}).get("ros__parameters", {})
    if "map_file_path" in params:
        resolved_path = os.path.expanduser(params["map_file_path"])
        params["map_file_path"] = resolved_path
        os.makedirs(os.path.dirname(resolved_path), exist_ok=True)

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(fast_lio_params, tmp)
    tmp.close()

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("fast_lio"), "launch", "mapping.launch.py"]
            )
        ),
        launch_arguments={
            "config_path": os.path.dirname(tmp.name),
            "config_file": os.path.basename(tmp.name),
            "rviz": rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Reused rather than duplicated, so the EKF node and its parameters are defined once.
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("autonomy"), "launch", "ekf.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": ekf_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz,
            declare_ekf_params_file,
            fast_lio_launch,
            ekf_launch,
        ]
    )
