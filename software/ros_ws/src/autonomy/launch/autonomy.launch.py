import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch the complete autonomy stack including SLAM and navigation.
    
    This launch file combines:
    - online_async_launch.py: SLAM Toolbox for mapping/localization
    - perseus_nav_bringup.launch.py: Nav2 navigation stack
    """
    
    autonomy_dir = get_package_share_directory("autonomy")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav_params_file = LaunchConfiguration("nav_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation/Gazebo clock",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the autonomy stack",
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            autonomy_dir, "config", "slam_toolbox_params.yaml"
        ),
        description="Full path to the ROS2 parameters file for SLAM Toolbox",
    )

    declare_nav_params_file_cmd = DeclareLaunchArgument(
        "nav_params_file",
        default_value=os.path.join(autonomy_dir, "config", "perseus_nav_params.yaml"),
        description="Full path to the ROS2 parameters file for Nav2",
    )

    # Include SLAM Toolbox launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autonomy_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "map_file_name": "$HOME/perseus-v2/software/ros_ws/src/autonomy/map/sim_map_serial", # FOR SIMULATION ONLY
        }.items(),
    )

    # Include Nav2 Bringup launch
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autonomy_dir, "launch", "perseus_nav_bringup.launch.py")
        ),
        launch_arguments={
            "namespace": "",
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": nav_params_file,
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_nav_params_file_cmd)

    # Include launch files
    ld.add_action(slam_launch)
    ld.add_action(nav_launch)

    return ld
