#!/usr/bin/env python3
"""Launch map_to_mesh.py: turn FAST-LIO's live map into a mesh, and stream it in.

Not a ROS node -- map_to_mesh.py is a one-shot CLI script (capture one message off the
map topic, convert, reconstruct, publish, exit), so this runs it as a plain subprocess
via ExecuteProcess rather than launch_ros's Node action, which expects a persistent,
package-registered ROS executable.

Needs FAST-LIO already running with publish.map_en:true (the default in
config/livox_mid360.yaml) -- there is nothing on the map topic to capture otherwise.
Re-run this (e.g. after driving further) any time you want move_base_flex to pick up
an updated mesh; it does not need restarting to receive one.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    script_path = os.path.join(
        get_package_share_directory("autonomy_bringup"), "scripts", "map_to_mesh.py"
    )

    declare_topic = DeclareLaunchArgument(
        "topic",
        default_value="/Laser_map",
        description="FAST-LIO topic publishing the accumulated map",
    )
    declare_timeout = DeclareLaunchArgument(
        "timeout",
        default_value="10.0",
        description="Seconds to wait for a message on topic before giving up",
    )
    declare_output = DeclareLaunchArgument(
        "output",
        default_value=os.path.expanduser("~/maps/mesh.ply"),
        description="Reconstructed mesh, for navigation.launch.py's mesh_map_path",
    )
    declare_voxel_size = DeclareLaunchArgument(
        "voxel_size",
        default_value="10.0",
        description="lvr2_reconstruct's -v/--voxelsize: smaller is finer but slower",
    )
    declare_point_cloud_manager = DeclareLaunchArgument(
        "point_cloud_manager",
        default_value="LVR2",
        description="lvr2_reconstruct's -p/--pcm: FLANN, PCL, LVR2, or LBVH_CUDA",
    )
    declare_publish_topic = DeclareLaunchArgument(
        "publish_topic",
        default_value="/move_base_flex/mesh_update",
        description="Topic to publish the reconstructed mesh on, for mesh_map's "
        "mesh_geometry_sub to hot-reload from",
    )
    declare_frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="odom",
        description="Frame the mesh is published in; must match mesh_map's global_frame "
        "(navigation.launch.py's global_frame argument)",
    )
    declare_publish_hold = DeclareLaunchArgument(
        "publish_hold",
        default_value="2.0",
        description="Seconds to keep the node alive after publishing, so the message "
        "actually gets sent before the process exits",
    )

    map_to_mesh = ExecuteProcess(
        cmd=[
            "python3",
            script_path,
            "--topic",
            LaunchConfiguration("topic"),
            "--timeout",
            LaunchConfiguration("timeout"),
            "--output",
            LaunchConfiguration("output"),
            "--voxel-size",
            LaunchConfiguration("voxel_size"),
            "--point-cloud-manager",
            LaunchConfiguration("point_cloud_manager"),
            "--publish-topic",
            LaunchConfiguration("publish_topic"),
            "--frame-id",
            LaunchConfiguration("frame_id"),
            "--publish-hold",
            LaunchConfiguration("publish_hold"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_topic,
            declare_timeout,
            declare_output,
            declare_voxel_size,
            declare_point_cloud_manager,
            declare_publish_topic,
            declare_frame_id,
            declare_publish_hold,
            map_to_mesh,
        ]
    )
