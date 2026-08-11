#!/usr/bin/env python3
"""Launch the nodes nav2's global costmap depends on, but that nav2 itself does not provide.

Two pieces, both parameterised by config/navigation.yaml:

  1. footprint_broadcaster's flat_footprint_broadcaster flattens odom -> base_link into
     odom -> base_footprint. nav2's costmaps and controller assume a robot that moves on a
     plane, but nothing publishes a planar base frame on its own: robot_localization only owns
     odom -> base_link, and that transform carries FAST-LIO's real z, roll and pitch
     (ekf_config.yaml runs with two_d_mode: false).

  2. global_traversability turns FAST-LIO's accumulated map cloud (/Laser_map) into a
     terrain-aware occupancy costmap, in place of a global costmap sourced only from a 2D SLAM
     map. Point nav2's global costmap static layer at its costmap topic once nav2 itself is
     brought up elsewhere.

localisation.launch.py must already be running to supply odom -> base_link and /Laser_map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("autonomy_bringup"), "config", "navigation.yaml"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the simulation clock. Only set true when something publishes "
        "/clock.",
    )

    flat_footprint_broadcaster_node = Node(
        package="footprint_broadcaster",
        executable="flat_footprint_broadcaster",
        name="flat_footprint_broadcaster",
        parameters=[config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    global_traversability_node = Node(
        package="global_traversability",
        executable="global_traversability",
        name="global_traversability",
        parameters=[config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            flat_footprint_broadcaster_node,
            global_traversability_node,
        ]
    )
