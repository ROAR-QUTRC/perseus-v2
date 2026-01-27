import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the complete autonomy stack including SLAM, navigation.

    This launch file combines:
    - online_async_launch.py: SLAM Toolbox for mapping/localization
    - perseus_nav_bringup.launch.py: Nav2 navigation stack
    """

    autonomy_dir = get_package_share_directory("autonomy")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav_params_file = LaunchConfiguration("nav_params_file")
    ekf_config_file = LaunchConfiguration("ekf_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation/Gazebo clock",
    )

    declare_imu_topic = DeclareLaunchArgument(
        "imu_topic",
        default_value="/livox/imu",
        description="IMU topic for robot_localization ekf (imu0)",
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the autonomy stack",
    )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(autonomy_dir, "config", "slam_toolbox_params.yaml"),
        description="Full path to the ROS2 parameters file for SLAM Toolbox",
    )

    map_file = os.path.join(autonomy_dir, "map", "sim_map_serial")

    declare_nav_params_file_cmd = DeclareLaunchArgument(
        "nav_params_file",
        default_value=os.path.join(autonomy_dir, "config", "nav_sim_params.yaml"),
        description="Full path to the ROS2 parameters file for Nav2",
    )

    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        "ekf_config_file",
        default_value=os.path.join(autonomy_dir, "config", "ekf_config.yaml"),
        description="Full path to the ROS2 parameters file for EKF",
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
            "map_file_name": map_file,  # FOR SIMULATION ONLY
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

    # Include Keepout Filter launch (optional)

    # EKF Node for sensor fusion and localization
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time, "imu0": LaunchConfiguration("imu_topic")},
        ],
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_imu_topic)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_nav_params_file_cmd)
    ld.add_action(declare_ekf_config_file_cmd)

    # Include launch files and nodes
    ld.add_action(ekf_node)
    ld.add_action(slam_launch)
    ld.add_action(nav_launch)

    return ld
