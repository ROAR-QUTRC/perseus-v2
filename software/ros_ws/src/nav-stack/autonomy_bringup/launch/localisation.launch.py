#!/usr/bin/env python3
"""Launch the localisation stack: FAST-LIO odometry fused by the EKF.

Three pieces, and the split matters:

  1. dust_filter (perseus_sensors) declutters the raw Livox cloud, removing airborne dust
     and sand -- see perseus_sensors/config/dust_filter.yaml. livox_mid360.yaml's
     common.lid_topic already reads its output (/livox/lidar/filtered), not the raw
     /livox/lidar, so FAST-LIO gets nothing at all until this is running.
  2. fast_lio provides LiDAR-inertial odometry from config/livox_mid360.yaml. It publishes
     /Odometry as odom -> base_link and broadcasts no TF of its own.
  3. ekf.launch.py runs robot_localization from config/ekf_config.yaml, fusing that pose
     with IMU angular velocity and owning the odom -> base_link transform.

Start order does not matter. fast_lio withholds /Odometry until lid_frame -> base_frame
resolves in TF, so until robot_state_publisher is up the EKF runs on the IMU alone.

use_composition (default true) loads dust_filter and FAST-LIO's LaserMappingNode into one
component container, so the filtered cloud passes between them by pointer instead of over
DDS. Pass use_composition:=false to fall back to two separate processes (also the only way
to get fast_lio's own RViz launch via the `rviz` argument, which the composed container does
not start). Composition needs a FAST-LIO build with the LaserMappingNode component
registered; if that has not landed yet, use use_composition:=false.

Map saving is handled inside fast_lio, via the pcd_save block of livox_mid360.yaml:
periodically, on shutdown, and on demand via the /map_save service.

The config is tuned for the real robot. Pass sim:=true against Gazebo, which publishes a
differently shaped cloud -- see sim_overrides() for exactly what changes and why.
"""

import os
import tempfile

from matplotlib import container
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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
    dust_filter_params_file = LaunchConfiguration("dust_filter_params_file")
    is_sim = LaunchConfiguration("sim").perform(context).lower() == "true"
    use_composition = (
        LaunchConfiguration("use_composition").perform(context).lower() == "true"
    )

    fast_lio_params_file = os.path.join(
        get_package_share_directory("autonomy_bringup"), "config", "livox_mid360.yaml"
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

    if use_composition:
        # dust_filter republishes /livox/lidar as /livox/lidar/filtered, which
        # livox_mid360.yaml's common.lid_topic already expects to read FAST-LIO's input
        # from. Loading both in one container lets that hop happen by pointer instead of
        # over DDS -- but only because use_intra_process_comms is set on both: a container
        # does not turn it on by itself, it defaults to off per node like anywhere else.
        fast_lio_launch = ComposableNodeContainer(
            name="localisation_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            composable_node_descriptions=[
                ComposableNode(
                    package="perseus_sensors",
                    plugin="perseus_sensors::DustFilter",
                    name="dust_filter",
                    parameters=[
                        dust_filter_params_file,
                        {"use_sim_time": use_sim_time},
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="fast_lio",
                    plugin="LaserMappingNode",
                    name="laser_mapping",
                    parameters=[tmp.name, {"use_sim_time": use_sim_time}],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
        )
    else:
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
                [FindPackageShare("autonomy_bringup"), "launch", "ekf.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": ekf_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Watches /odometry/filtered against /cmd_vel_out for wheel slip, so it belongs
    # wherever the EKF that produces /odometry/filtered is brought up.
    watchdog_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("watchdog"), "launch", "mobility_watchdog.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return [fast_lio_launch, ekf_launch, watchdog_launch]


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
            [FindPackageShare("autonomy_bringup"), "config", "ekf_config.yaml"]
        ),
        description="Parameters file for the robot_localization EKF.",
    )

    declare_use_composition = DeclareLaunchArgument(
        "use_composition",
        default_value="true",
        description="Load dust_filter and FAST-LIO's LaserMappingNode into one component "
        "container instead of two separate processes. Needs a FAST-LIO build with the "
        "LaserMappingNode component registered; set false to fall back to fast_lio's own "
        "mapping.launch.py (also the only way to get its `rviz` argument).",
    )

    declare_dust_filter_params_file = DeclareLaunchArgument(
        "dust_filter_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("perseus_sensors"), "config", "dust_filter.yaml"]
        ),
        description="Parameters file for dust_filter, used only when use_composition:=true.",
    )
    bias_remover_container = ComposableNodeContainer(
        name="imu_bias_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # mt is nice for multiple callbacks
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="perseus_sensors",
                plugin="imu_processors::BiasEstimator",
                name="imu_bias_estimator",
                parameters=[
                    {
                        "use_odom": True,
                        "use_cmd_vel": False,
                        "accumulator_alpha": 0.01,
                        "stationary_mode": "AND",  # OR / AND
                        "imu_in_topic": "/livox/imu",
                        "odom_topic": "/odom", # from the wheel encoder
                        "bias_out_topic": "/livox/gyro_bias",
                        "estimator_rate_hz": 100.0,
                    }
                ],
            ),
            ComposableNode(
                package="perseus_sensors",
                plugin="imu_processors::BiasRemover",
                name="imu_bias_remover",
                parameters=[
                    {
                        "imu_in_topic": "/livox/imu",
                        "bias_in_topic": "/livox/gyro_bias",
                        "imu_out_topic": "/livox/imu/corrected",
                        "output_rate_hz": 100.0,
                    }
                ],
            ),
        ],
    )
    return LaunchDescription(
        [
            declare_sim,
            declare_use_sim_time,
            declare_rviz,
            declare_ekf_params_file,
            declare_use_composition,
            declare_dust_filter_params_file,
            bias_remover_container,
            OpaqueFunction(function=launch_setup),
        ]
    )
