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

The config is tuned for the real robot. Pass sim:=true against Gazebo, which publishes a
differently shaped cloud -- see sim_overrides() for exactly what changes and why.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def sim_overrides(params):
    """Adjust the real-robot parameters for Gazebo's point cloud, in place.

    Gazebo publishes x,y,z,intensity,ring. The real Livox driver publishes
    reflectivity,tag,line. lidar_type selects which of those layouts FAST-LIO deserialises
    into, so against the sim, type 4 finds no `line` field and reads the scan-line index as 0
    for every point -- collapsing all points onto one line and wrecking the per-point
    timestamps that motion compensation depends on. Type 2 reads `ring`, which the sim does
    provide, and derives times from azimuth when no per-point time is present.

    scan_line is the number of lines to expect: 32 vertical samples in the sim, against 4 real
    scan lines on a MID-360. The Velodyne handler skips points whose ring exceeds it.

    lid_topic needs no override: Gazebo's ±Inf no-return rays are rejected by the preprocess
    handlers now, so the raw topic is safe to consume directly.
    """
    params["preprocess"]["lidar_type"] = 2
    params["preprocess"]["scan_line"] = 32


def launch_setup(context, *args, **kwargs):
    """Build the actions once the launch arguments can be resolved.

    An OpaqueFunction is needed because the FAST-LIO config has to be read, adjusted and
    rewritten as a real file before the node starts: fast_lio takes a path, not a
    substitution, so ~ in map_file_path and the sim overrides both have to be applied here
    rather than deferred.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    is_sim = LaunchConfiguration("sim").perform(context).lower() == "true"

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

    if is_sim:
        sim_overrides(params)

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

    return [fast_lio_launch, ekf_launch]


def generate_launch_description():
    declare_sim = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="Adapt the LiDAR parameters to Gazebo's point cloud. You almost always "
        "want use_sim_time:=true alongside it.",
    )

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

    return LaunchDescription(
        [
            declare_sim,
            declare_use_sim_time,
            declare_rviz,
            declare_ekf_params_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
