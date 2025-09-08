from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    hardware_plugin = LaunchConfiguration("hardware_plugin", default="mock_components/GenericSystem")
    can_bus = LaunchConfiguration("can_bus", default="")

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
            "hardware_plugin": hardware_plugin,
            "can_bus": can_bus,
        }.items(),
    )

    # RViz with nixGL support
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("perseus_description"), "rviz", "view_perseus.rviz"]
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

    # Joint State Publisher GUI
    joint_state_publisher_gui = ExecuteProcess(
        cmd=["ros2", "run", "joint_state_publisher_gui", "joint_state_publisher_gui"],
        output="screen",
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

    return LaunchDescription(
        [
            rsp_launch,
            rviz,
            joint_state_publisher_gui,
            # controllers_launch,
        ]
    )
