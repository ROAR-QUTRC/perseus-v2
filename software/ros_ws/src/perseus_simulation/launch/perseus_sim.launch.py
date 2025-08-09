from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time")
    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock",
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
        }.items(),
    )

    # RViz with nixGL support
    rviz = ExecuteProcess(
        cmd=[
            "nix",
            "run",
            "--impure",
            "github:nix-community/nixGL",
            "--",
            "rviz2",
            # "-d", rviz_config,  # If you want to use RViz config, uncomment this and set rviz_config properly
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

    launch_files = [
        gz_launch,
        rviz,  # Start RViz with nixGL support
        rsp_launch,  # Robot state publisher
        controllers_launch,  # Controllers
    ]

    return LaunchDescription(arguments + launch_files)
