#!/usr/bin/env python3
"""Launch the mesh navigation stack: Move Base Flex planning and control over a mesh map.

mbf_mesh_nav (from the mesh_navigation bundle) provides the actual move_base_flex node.
It plans with cvp_mesh_planner and drives with mesh_controller over a layered mesh_map --
see config/mesh_nav_params.yaml for the layer graph and plugin settings.

Needs a mesh already on disk: mesh_map_path and mesh_map_working_path both default to
~/maps/mesh_map.h5, mirroring FAST-LIO's map_file_path convention (config/livox_mid360.yaml).
Loading and working file can be the same path -- mesh_map computes the layer graph on load
and persists it back into mesh_working_file, so the second launch onward reuses cached
per-vertex costs instead of recomputing them. Pass a different mesh_map_path to load an
existing mesh (e.g. one built elsewhere) while still caching costs to your own working copy.

Defaults to planning directly against odom -> base_link (from the EKF, see ekf.launch.py),
since that's the one frame this project's autonomy stack always has. Pass
global_frame:=map once slam_toolbox (perseus_nav_params.yaml) is running and publishing
map -> odom, to plan in a frame that survives odometry drift.

move_base_flex publishes cmd_vel under its own node name; cmd_vel_topic remaps that to
cmd_vel_mux's "Navigation stack velocity commands" input (config/cmd_vel_mux.yaml) rather
than driving the robot directly, so it arbitrates with teleop the same way nav2 would.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_mesh_path = os.path.expanduser("~/maps/mesh_map.h5")
    os.makedirs(os.path.dirname(default_mesh_path), exist_ok=True)

    declare_mesh_map_path = DeclareLaunchArgument(
        "mesh_map_path",
        default_value=default_mesh_path,
        description="Mesh file to load. Our internal HDF5 format or anything Assimp can "
        "read (OBJ, PLY, STL, ...).",
    )
    declare_mesh_map_working_path = DeclareLaunchArgument(
        "mesh_map_working_path",
        default_value=default_mesh_path,
        description="Where computed layer costs are cached. Must be HDF5. Same as "
        "mesh_map_path by default, so costs persist across launches instead of being "
        "recomputed from a read-only input mesh.",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("autonomy_bringup"), "config", "mesh_nav_params.yaml"]
        ),
        description="Full path to the ROS2 parameters file for move_base_flex/mesh_map.",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true. Requires something to publish /clock.",
    )
    declare_global_frame = DeclareLaunchArgument(
        "global_frame",
        default_value="odom",
        description="Frame the mesh is loaded into and planned in. Needs a TF publisher "
        "for this frame -> robot_frame; pass 'map' once slam_toolbox is running.",
    )
    declare_cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel_nav",
        description="Topic move_base_flex's velocity commands are remapped to.",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    mesh_nav_server = Node(
        name="move_base_flex",
        package="mbf_mesh_nav",
        executable="mbf_mesh_nav",
        output="screen",
        remappings=[
            ("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
        ],
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "global_frame": LaunchConfiguration("global_frame"),
                "mesh_map.mesh_file": LaunchConfiguration("mesh_map_path"),
                "mesh_map.mesh_working_file": LaunchConfiguration(
                    "mesh_map_working_path"
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            declare_mesh_map_path,
            declare_mesh_map_working_path,
            declare_params_file,
            declare_use_sim_time,
            declare_global_frame,
            declare_cmd_vel_topic,
            mesh_nav_server,
        ]
    )
