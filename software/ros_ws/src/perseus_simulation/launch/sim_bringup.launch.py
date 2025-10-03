from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
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
    rviz_config = LaunchConfiguration("rviz_config")
    autostart = LaunchConfiguration("autostart")
    slam_params_file = LaunchConfiguration("slam_params_file")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="view.rviz",
            description="RVIZ config file to use (view.rviz or view_no_image.rviz for shader issue workaround)",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the slamtoolbox. "
            "Ignored when use_lifecycle_manager is true.",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("autonomy"), "config", "mapper_params_online_async.yaml"]
            ),
            description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
        ),
    ]

    # LAUNCH PERSEUS SIMULATION
    perseus_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_simulation"),
                        "launch",
                        "perseus_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config,
        }.items(),
    )

    # LAUNCH AUTONOMY ONLINE ASYNC (SLAM) - Delayed by 20 seconds after perseus_sim
    autonomy_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("autonomy"),
                        "launch",
                        "online_async_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # LAUNCH NAVIGATION STACK - Delayed by 25 seconds to allow SLAM to initialize
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("autonomy"),
                        "launch",
                        "perseus_nav_bringup.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    # Delay SLAM launch by 20 seconds to allow perseus_sim to fully initialize
    slam_delay = TimerAction(
        period=20.0,  # Wait 20 seconds after perseus_sim launch for full initialization
        actions=[autonomy_slam_launch],
    )

    # Delay Navigation launch by 25 seconds to allow SLAM to initialize first
    nav_delay = TimerAction(
        period=25.0,  # Wait 25 seconds after perseus_sim launch for SLAM initialization
        actions=[nav_launch],
    )

    launch_files = [
        perseus_sim_launch,
        slam_delay,
        nav_delay,
    ]

    return LaunchDescription(arguments + launch_files)
