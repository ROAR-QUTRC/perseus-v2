"""
RealSense camera launch (+ image compression).

Every RealSense driver setting lives in `config/realsense.yaml`. This launch file
deliberately exposes nothing but the stream toggles, since those are the only settings
worth changing per run:

    enable_depth       depth stream
    enable_infra_pair  the infra1/infra2 IR pair (infra0 is a config file setting)
    enable_color       color stream

The defaults match the baseline setup: color and depth on, IR off.

Usage:
    ros2 launch perseus_sensors realsense.launch.py

    ros2 launch perseus_sensors realsense.launch.py \
        enable_infra_pair:=true enable_color:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CONFIG_FILE_NAME = "realsense.yaml"

# Topics for the image_transport republishers. These follow from camera_name and
# camera_namespace in the config file, so they are fixed here rather than exposed.
RGB_IMAGE_TOPIC = "/camera/camera/color/image_raw"
RGB_COMPRESSED_TOPIC = "/camera/camera/color/image_compressed"
DEPTH_IMAGE_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
DEPTH_COMPRESSED_TOPIC = "/camera/camera/depth/image_compressedDepth"


def generate_launch_description():
    """Build the launch description for the RealSense driver and its republishers."""
    config_file = str(
        Path(get_package_share_directory("perseus_sensors"))
        / "config"
        / CONFIG_FILE_NAME
    )

    enable_depth_arg = DeclareLaunchArgument(
        "enable_depth", default_value="true", description="Enable the depth stream"
    )
    enable_infra_pair_arg = DeclareLaunchArgument(
        "enable_infra_pair",
        default_value="false",
        description="Enable the infra1/infra2 IR stream pair",
    )
    enable_color_arg = DeclareLaunchArgument(
        "enable_color", default_value="true", description="Enable the color stream"
    )

    # realsense2_camera's rs_launch.py passes its node `parameters=[args, config_file]`,
    # so any setting present in the YAML wins over the matching launch argument. The
    # stream toggles are therefore kept out of the YAML entirely and supplied here.
    #
    # The include is scoped with forwarding=False so that exactly these configurations
    # reach it. IncludeLaunchDescription does not scope on its own, so without the group
    # every configuration declared here would leak into rs_launch.py, which since v4.58
    # warns about each one it does not recognise.
    realsense = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("realsense2_camera"),
                            "launch",
                            "rs_launch.py",
                        ]
                    )
                )
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            "config_file": config_file,
            "enable_depth": LaunchConfiguration("enable_depth"),
            "enable_color": LaunchConfiguration("enable_color"),
            # infra1 and infra2 are the left/right halves of one stereo pair; the driver
            # takes them as separate flags but they are only useful together.
            "enable_infra1": LaunchConfiguration("enable_infra_pair"),
            "enable_infra2": LaunchConfiguration("enable_infra_pair"),
        },
    )

    rgb_compress = Node(
        package="image_transport",
        executable="republish",
        name="rgb_compress_republish",
        output="screen",
        arguments=["raw", "compressed"],
        remappings=[("in", RGB_IMAGE_TOPIC), ("out", RGB_COMPRESSED_TOPIC)],
    )

    depth_compress = Node(
        package="image_transport",
        executable="republish",
        name="depth_compress_republish",
        output="screen",
        arguments=["raw", "compressedDepth"],
        remappings=[("in", DEPTH_IMAGE_TOPIC), ("out", DEPTH_COMPRESSED_TOPIC)],
    )

    return LaunchDescription(
        [
            enable_depth_arg,
            enable_infra_pair_arg,
            enable_color_arg,
            realsense,
            rgb_compress,
            depth_compress,
        ]
    )
