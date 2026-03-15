from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    AndSubstitution,
    NotEqualsSubstitution,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import FrontendLaunchDescriptionSource


def generate_launch_description():
    domain_id_autonomy = LaunchConfiguration("domain_id_autonomy")
    domain_id_main = LaunchConfiguration("domain_id_main")
    config_filename = LaunchConfiguration("config_filename")
    arguments = [
        DeclareLaunchArgument(
            "domain_id_main",
            default_value="",
            description="ROS_DOMAIN_ID of this device (to_domain). Rover=0, Main=2",
        ),
        DeclareLaunchArgument(
            "domain_id_autonomy",
            default_value="",
            description="ROS_DOMAIN_ID of this device (from_domain). Rover=0, Main=2",
        ),
        DeclareLaunchArgument(
            "config_filename",
            default_value="big-brain-bridge.yaml",
            description="Config file for the domain bridge - inside perseus_domain_bridge/config",
        ),
    ]
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("perseus_domain_bridge"),
            "config",
            config_filename,
        ],
    )
    remap_ids_dict = {
        "--from": domain_id_autonomy,
        "--to": domain_id_main,
        "config": config_file,
    }
    domain_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("domain_bridge"),
                        "launch",
                        "domain_bridge.launch.xml",
                    ],
                ),
            ]
        ),
        launch_arguments=IfElseSubstitution(
            AndSubstitution(
                NotEqualsSubstitution(domain_id_autonomy, ""),
                NotEqualsSubstitution(domain_id_main, ""),
            ),
            remap_ids_dict.items(),
            {"config": config_file}.items(),
        ),
    )
    launch_files = [domain_bridge_launch]
    return LaunchDescription([arguments, launch_files])
