from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import UnlessCondition 
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
    wheel_odom_only = LaunchConfiguration("wheel_odom_only", default="false") # Disabled - using EKF for odometry transforms


    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock",
        ),
        DeclareLaunchArgument(
            "wheel_odom_only",
            default_value="false",
            description="If false, use fused odometry sensor (EKF). If true, use wheel odometry only.",
        ),
    ]
    # IMPORTED LAUNCH FILES
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_simulation"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus"),
                        "launch",
                        "robot_state_publisher.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "hardware_plugin": "gz_ros2_control/GazeboSimSystem",
        }.items(),
    )
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus"),
                        "launch",
                        "controllers.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "launch_controller_manager": "false",
            "enable_odom_tf": wheel_odom_only,
        }.items(),
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "rviz", "view.rviz"]
    )
    ekf_config_file = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "config", "ekf_sim_config.yaml"]
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

    # EKF node - only run if wheel_odom_only is FALSE
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
    # Add delay to EKF to ensure all other nodes are ready
    ekf_delayed = TimerAction(
        period=5.0,
        actions=[ekf_node],
        condition=UnlessCondition(wheel_odom_only),  # Only run if wheel_odom_only is false
    )
    launch_files = [
        gz_launch,
        rsp_launch,
        controllers_launch,
        ekf_delayed,
        rviz,
    ]

    return LaunchDescription(arguments + launch_files)
