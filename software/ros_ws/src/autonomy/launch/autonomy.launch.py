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
    nav_params_file = LaunchConfiguration("nav_params_file")
    ekf_config_file = LaunchConfiguration("ekf_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_yaml_file = LaunchConfiguration("map")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation/Gazebo clock",
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the autonomy stack",
    )

    declare_nav_params_file_cmd = DeclareLaunchArgument(
        "nav_params_file",
        default_value=os.path.join(autonomy_dir, "config", "nav_aut_arch.yaml"),
        description="Full path to the ROS2 parameters file for Nav2",
    )

    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        "ekf_config_file",
        default_value=os.path.join(
            autonomy_dir, "config", "ekf_config_stereo_imu.yaml"
        ),
        # default_value=os.path.join(autonomy_dir, "config", "ekf_config.yaml"),
        description="Full path to the ROS2 parameters file for EKF",
    )

    declare_map_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(autonomy_dir, "maps", "aut_arch_2026.yaml"),
        description="Full path to the map yaml file",
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

    # EKF Node for sensor fusion and localization
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Map Server Node
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml_file,
                "frame_id": "map",
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # Lifecycle Manager for map_server
    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "autostart": autostart,
                "node_names": ["map_server"],
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # Static transform: map -> odom (identity, FAST-LIO2 handles drift correction)
    static_map_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_nav_params_file_cmd)
    ld.add_action(declare_ekf_config_file_cmd)
    ld.add_action(declare_map_cmd)

    # Include launch files and nodes
    ld.add_action(ekf_node)
    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_manager)
    ld.add_action(static_map_odom_tf)
    ld.add_action(nav_launch)
    return ld
