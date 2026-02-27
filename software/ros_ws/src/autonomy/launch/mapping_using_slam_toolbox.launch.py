from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    launch_bringup = LaunchConfiguration("launch_bringup")

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
        DeclareLaunchArgument(
            "launch_bringup",
            default_value="true",
            description="Launch robot bringup (set to false if already running)",
        ),
    ]

    # Include robot bringup from perseus package
    perseus_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("perseus"), "launch", "perseus.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(launch_bringup),
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

    # RViz with nixGL support
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("autonomy"), "rviz", "perseus_slam.rviz"]
    )
    rviz = ExecuteProcess(
        cmd=[
            "nix",
            "run",
            "--impure",
            "github:nix-community/nixGL",
            "--",
            "rviz2",
            "-d",
            rviz_config,
        ],
        output="screen",
        additional_env={
            "NIXPKGS_ALLOW_UNFREE": "1",
            "QT_QPA_PLATFORM": "xcb",
            "QT_SCREEN_SCALE_FACTORS": "1",
            "ROS_NAMESPACE": "/",
            "RMW_QOS_POLICY_HISTORY": "keep_last",
            "RMW_QOS_POLICY_DEPTH": "100",
        },
    )

    nodes = [
        perseus_bringup,
        static_tf_publisher,
        slam_toolbox,
        rviz,
    ]

    return LaunchDescription(declare_args + nodes)
