from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_yaml = os.path.join(
        get_package_share_directory("perseus_sensors"),
        "config",
        "realsense_params.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "rs_launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_yaml,
            description="YAML parameter file for realsense2_camera",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                "params_file": params_file,

                # Optional if you want to control naming here instead of YAML:
                # "camera_name": "camera",
                # "camera_namespace": "camera",
            }.items(),
        ),
    ])
