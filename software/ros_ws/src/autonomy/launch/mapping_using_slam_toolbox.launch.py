import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    declare_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation/Gazebo clock",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("autonomy"), "config", "slam_toolbox_params.yaml"]
            ),
            description="Full path to the ROS2 parameters file for SLAM Toolbox",
        ),
    ]

    # Include robot bringup from perseus_lite package
    perseus_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_lite"),
                        "launch",
                        "perseus_lite.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # SLAM nodes
    static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom_publisher",
        # arguments=["0", "0", "0", "0", "0", "0", "base_link", "lidar_frame"],
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )
    # Node(
    # # Base link to LIDAR transform
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="lidar_tf_publisher",
    #     arguments=["0", "0", "0.2", "0", "0", "0", "odom", "lidar_frame"],
    # ),

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            #     get_package_share_directory("autonomy")
            #     + "/config/slam_toolbox_params.yaml",
            slam_params_file,
            # {
            #     "use_sim_time": use_sim_time,
            #     "odom_frame": "odom",
            #     "base_frame": "base_link",
            #     "map_frame": "map",
            #     "scan_topic": "/scan",
            # },
        ],
    )

    # RViz
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("autonomy"), "rviz", "perseus_slam.rviz"]
    )
    rviz_env = {
        "QT_QPA_PLATFORM": "xcb",
        "QT_SCREEN_SCALE_FACTORS": "1",
        "ROS_NAMESPACE": "/",
        "RMW_QOS_POLICY_HISTORY": "keep_last",
        "RMW_QOS_POLICY_DEPTH": "100",
    }
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        # rviz2 (Qt) can crash in libfontconfig if it shares ~/.cache with
        # the system's fontconfig (different, ABI-incompatible version).
        rviz_env["XDG_CACHE_HOME"] = os.path.join(conda_prefix, "var", "cache")
    rviz = ExecuteProcess(
        cmd=[
            "rviz2",
            "-d",
            rviz_config,
        ],
        output="screen",
        additional_env=rviz_env,
    )

    nodes = [
        perseus_bringup,
        static_tf_publisher,
        slam_toolbox,
        rviz,
    ]

    return LaunchDescription(declare_args + nodes)
