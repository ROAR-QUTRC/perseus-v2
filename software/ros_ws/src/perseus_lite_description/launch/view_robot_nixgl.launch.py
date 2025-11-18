#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directories
    perseus_lite_pkg = get_package_share_directory("perseus_lite")
    perseus_lite_desc_pkg = get_package_share_directory("perseus_lite_description")

    # Path to the main xacro file (from perseus_lite package)
    xacro_file = os.path.join(perseus_lite_pkg, "urdf", "perseus_lite.urdf.xacro")

    # Process the xacro file with default arguments
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " hardware_plugin:=mock_components/GenericSystem",  # Use mock hardware for visualization
                " serial_port:=/dev/null",
                " baud_rate:=115200",
            ]
        ),
        value_type=str,
    )

    # Path to RViz config
    rviz_config_file = os.path.join(perseus_lite_desc_pkg, "rviz", "view_robot.rviz")

    return LaunchDescription(
        [
            # Robot State Publisher - publishes TF transforms based on URDF
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": False}
                ],
            ),
            # Joint State Publisher GUI - allows manual control of joints
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            # RViz2 with nixgl wrapper - for visualization on non-NixOS systems
            ExecuteProcess(
                cmd=["nixgl", "rviz2", "-d", rviz_config_file], output="screen"
            ),
        ]
    )
