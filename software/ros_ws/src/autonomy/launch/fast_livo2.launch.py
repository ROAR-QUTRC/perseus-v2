#!/usr/bin/env python3
"""Launch FAST-LIVO2 (LiDAR-inertial-visual odometry) for the Perseus stack.

FAST-LIVO2's visual front-end (via vikit_ros) reads the camera intrinsics from a
global parameter server, so this launch brings up three things:

  1. ``parameter_blackboard`` (demo_nodes_cpp) loaded with the *camera* params -
     this is the "global parameter server" vikit_ros queries for intrinsics.
  2. ``fastlivo_mapping`` (the fast_livo node) loaded with the *sensor/mapping*
     params (LiDAR + IMU + voxel-map settings).
  3. Optional RViz, and an optional compressed->raw image republisher.

Defaults point at the config shipped in the ``fast_livo`` package (Livox Avia +
pinhole camera). Override ``mapping_params_file`` / ``camera_params_file`` to
point at Perseus-specific YAMLs (e.g. ones dropped into ``autonomy/config``).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fast_livo_share = FindPackageShare("fast_livo")

    # --- ARGUMENTS ---
    mapping_params_file = LaunchConfiguration("mapping_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_respawn = LaunchConfiguration("use_respawn")
    republish_image = LaunchConfiguration("republish_image")

    arguments = [
        DeclareLaunchArgument(
            "mapping_params_file",
            default_value=PathJoinSubstitution(
                [fast_livo_share, "config", "avia.yaml"]
            ),
            description="LiDAR/IMU/mapping parameters for the fastlivo_mapping node.",
        ),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=PathJoinSubstitution(
                [fast_livo_share, "config", "camera_pinhole.yaml"]
            ),
            description="Camera intrinsics served via parameter_blackboard for vikit_ros.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz with the FAST-LIVO2 config.",
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="true",
            description="Respawn nodes if they crash.",
        ),
        DeclareLaunchArgument(
            "republish_image",
            default_value="false",
            description="Republish /left_camera/image from compressed to raw "
            "(enable if the camera only publishes a compressed stream).",
        ),
    ]

    # --- NODES ---
    # Global parameter server holding the camera intrinsics (queried by vikit_ros).
    parameter_blackboard = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="parameter_blackboard",
        parameters=[camera_params_file],
        output="screen",
    )

    # Optional compressed -> raw image bridge.
    image_republisher = Node(
        condition=IfCondition(republish_image),
        package="image_transport",
        executable="republish",
        name="republish",
        arguments=["compressed", "raw"],
        remappings=[
            ("in/compressed", "/left_camera/image/compressed"),
            ("out", "/left_camera/image"),
        ],
        output="screen",
        respawn=use_respawn,
    )

    # The FAST-LIVO2 mapping node.
    fastlivo_mapping = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[mapping_params_file],
        output="screen",
        respawn=use_respawn,
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [fast_livo_share, "rviz_cfg", "fast_livo2.rviz"]
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        arguments
        + [
            parameter_blackboard,
            image_republisher,
            fastlivo_mapping,
            rviz,
        ]
    )
