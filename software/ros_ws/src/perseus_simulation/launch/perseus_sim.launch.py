from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os  

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
    # RViz configuration file
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "rviz", "rviz.rviz"]
    )

    ekf_config_file = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "config", "ekf_config.yaml"]
    )
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
    teleop_keyboard_controller = Node(
        package="perseus_teleop",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        prefix="gnome-terminal --geometry=60x20 -- ",  # Use gnome-terminal instead of xterm
        remappings=[
            # Example: remap cmd_vel topic if needed
            # ('/cmd_vel_out', '/your_robot/cmd_vel')
        ],
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
    # Add delay to controllers
    controllers_delayed = TimerAction(
        period=30.0,  # Wait 30 seconds for Gazebo to fully start
        actions=[controllers_launch]
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_file],
        remappings=[('/odometry/filtered', '/odom')] # Remap output to /odom
    )
    launch_files = [
        gz_launch,
        rsp_launch,  # Robot state publisher
        teleop_keyboard_controller,
        controllers_delayed,  # Controllers
        rviz,  # Start RViz with nixGL support
        ekf_node,  # EKF node
    ]

    return LaunchDescription(arguments + launch_files)
