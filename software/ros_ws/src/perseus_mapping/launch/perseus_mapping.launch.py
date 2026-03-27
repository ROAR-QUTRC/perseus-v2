from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import yaml
import tempfile


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("perseus_mapping"),
        "config",
    )
    launch_config_file = os.path.join(config_path, "launch_config.yaml")
    with open(launch_config_file, "r") as f:
        launch_config = yaml.safe_load(f)

    use_sim_time = str(launch_config.get("use_sim_time", "false")).lower()
    rviz = str(launch_config.get("rviz", "false")).lower()
    sim = str(launch_config.get("sim", "false")).lower()

    # Load and resolve ~ in map_file_path before passing to fast_lio
    ros_config_file = os.path.join(config_path, "livox_mid360.yaml")
    with open(ros_config_file, "r") as f:
        ros_config = yaml.safe_load(f)

    params = ros_config.get("/**", {}).get("ros__parameters", {})
    if "map_file_path" in params:
        resolved_path = os.path.expanduser(params["map_file_path"])
        params["map_file_path"] = resolved_path
        # Ensure the maps directory exists
        os.makedirs(os.path.dirname(resolved_path), exist_ok=True)

    # Write resolved config to a temp file for fast_lio to consume
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(ros_config, tmp)
    tmp.close()
    resolved_config_path = os.path.dirname(tmp.name)
    resolved_config_file = os.path.basename(tmp.name)

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fast_lio"),
                    "launch",
                    "mapping.launch.py",
                ]
            )
        ),
        launch_arguments={
            "config_path": resolved_config_path,
            "config_file": resolved_config_file,
            "rviz": rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    sim_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("perseus_mapping"),
                    "launch",
                    "sim_pointcloud_filter.launch.py",
                ]
            )
        ),
        condition=IfCondition(sim),
    )

    # Node for calling the fast-lio service topic to save the /cloud_registered map
    map_saver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("perseus_mapping"),
                    "launch",
                    "map_saver.launch.py",
                ]
            )
        ),
    )
    odom_transform_node = Node(
        package='perseus_mapping',
        executable='odom_transform',
        name='odom_transform',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        sim_filter_launch,
        fast_lio_launch,
        odom_transform_node,
        map_saver_launch,
    ])