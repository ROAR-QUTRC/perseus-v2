#!/usr/bin/env python3
"""Launch the robot_localization EKF on its own.

Fuses wheel odometry and IMU into a single state estimate and owns the
odom -> base_link transform. Parameters come from autonomy/config/ekf_config.yaml.

The IMU is consumed in its own frame: robot_localization looks up the transform from the
message's frame_id to base_link_frame and rotates the data itself, so the sensor mounting
is handled without anything in between.

The node is named ekf_filter_node to match the top-level key in that YAML:
robot_localization only picks up parameters whose key matches the node name, so
renaming the node here silently leaves it running on library defaults.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("autonomy"), "config", "ekf_config.yaml"]
        ),
        description="Full path to the ROS2 parameters file for the robot_localization EKF",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true. Requires something to publish /clock, "
        "otherwise the EKF waits at startup and never runs.",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([declare_use_sim_time, declare_params_file, ekf_node])
