#!/usr/bin/env python3
"""Turn FAST-LIO's live map into a mesh mesh_navigation can load, and stream it in.

publish_map() in laserMapping.cpp appends every scan to pcl_wait_pub -- the same
buffer /map_save persists to disk -- and republishes the whole accumulated cloud on
/Laser_map each time (gated by publish.map_en, already true in livox_mid360.yaml). So
one capture off that topic already is the whole map so far; nothing here needs to
accumulate scans itself.

Captured points get written out as .ply (lvr2_reconstruct's --inputFile takes
.pts/.xyz/.ply/.h5, not a live topic or PCL's format), then lvr2_reconstruct
surface-reconstructs that into a triangle mesh, written to --output.

That mesh is then also published as a mesh_msgs/MeshGeometryStamped on --publish-topic
(default /move_base_flex/mesh_update, transient_local so a subscriber already running
gets it reliably): mesh_map's mesh_geometry_sub (see
third-party-packages/mesh-navigation/patches/mesh-map-streamed-reload.patch) picks this
up and hot-reloads, so a running navigation.launch.py does not need restarting to pick
up a freshly reconstructed map. --output still gets written too, both because
navigation.launch.py's mesh_map_path needs a file to load from on its *first* startup,
and as a record independent of whether anything was listening on --publish-topic.

Needs rclpy, sensor_msgs_py, mesh_msgs, open3d (to read the reconstructed mesh back in
-- present in the project's dev shell, see nix/workspaces.nix), and lvr2_reconstruct
(from the lvr2 nix package) all on PATH -- run this from the project's dev shell, with
FAST-LIO already running.
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time
import uuid as uuid_module

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from mesh_msgs.msg import MeshGeometry, MeshGeometryStamped, MeshTriangleIndices
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import open3d as o3d

MESH_UPDATE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class _MapToMesh(Node):
    """Captures one map cloud, then (once the mesh is ready) publishes it once."""

    def __init__(self, capture_topic, publish_topic):
        super().__init__("map_to_mesh")
        self.cloud_msg = None
        self.create_subscription(PointCloud2, capture_topic, self._on_cloud, 1)
        self.mesh_pub = self.create_publisher(
            MeshGeometryStamped, publish_topic, MESH_UPDATE_QOS
        )

    def _on_cloud(self, msg):
        self.cloud_msg = msg


def capture_map_cloud(node, timeout_sec):
    # Reset first: on a repeat cycle, node.cloud_msg still holds the *previous* cycle's
    # cloud, and the wait loop below would otherwise see it as already-arrived and
    # return immediately without ever waiting for a fresh one.
    node.cloud_msg = None
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
    while node.cloud_msg is None:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.get_clock().now() > deadline:
            raise TimeoutError(
                f"No message received within {timeout_sec}s -- is FAST-LIO running "
                "with publish.map_en enabled?"
            )
    return node.cloud_msg


def write_ply(cloud_msg, ply_path):
    points = point_cloud2.read_points(
        cloud_msg, field_names=("x", "y", "z"), skip_nans=True
    )
    if len(points) == 0:
        raise ValueError(
            "Captured map has no points -- has FAST-LIO mapped anything yet?"
        )
    with open(ply_path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for p in points:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")


def reconstruct_mesh(ply_path, output_path, voxel_size, point_cloud_manager):
    output_dir = os.path.dirname(output_path) or "."
    os.makedirs(output_dir, exist_ok=True)
    subprocess.run(
        [
            "lvr2_reconstruct",
            "--inputFile",
            ply_path,
            "--outputDirectory",
            output_dir,
            "--outputFile",
            os.path.basename(output_path),
            "--voxelsize",
            str(voxel_size),
            "--pcm",
            point_cloud_manager,
        ],
        check=True,
    )


def mesh_file_to_geometry_msg(mesh_path, frame_id, stamp):
    """Reads a reconstructed mesh back in and builds the message mesh_map expects.

    lvr2_reconstruct is a subprocess, not a library call, so its output has to be read
    back off disk -- there is no in-memory buffer to hand off directly.
    """
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    if not mesh.has_vertices() or not mesh.has_triangles():
        raise ValueError(
            f"{mesh_path} has no vertices/triangles -- did reconstruction fail?"
        )
    mesh.compute_vertex_normals()

    vertices = np.asarray(mesh.vertices)
    normals = np.asarray(mesh.vertex_normals)
    triangles = np.asarray(mesh.triangles)

    geometry = MeshGeometry()
    geometry.vertices = [
        Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in vertices
    ]
    geometry.vertex_normals = [
        Point(x=float(n[0]), y=float(n[1]), z=float(n[2])) for n in normals
    ]
    geometry.faces = [
        MeshTriangleIndices(vertex_indices=[int(t[0]), int(t[1]), int(t[2])])
        for t in triangles
    ]

    msg = MeshGeometryStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.uuid = str(uuid_module.uuid4())
    msg.mesh_geometry = geometry
    return msg


def publish_mesh_update(node, msg, hold_sec):
    node.mesh_pub.publish(msg)
    # transient_local's durability caching lives in this process; a subscriber that's
    # already running (the normal case -- move_base_flex stays up while this gets
    # re-triggered) needs a moment for the message to actually go out before we exit.
    deadline = time.time() + hold_sec
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic",
        default="/Laser_map",
        help="FAST-LIO topic publishing the accumulated map (default: %(default)s)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for a message on --topic before giving up (default: %(default)s)",
    )
    parser.add_argument(
        "--output",
        default=os.path.expanduser("~/maps/mesh.ply"),
        help="Reconstructed mesh, for navigation.launch.py's mesh_map_path (default: %(default)s)",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=10.0,
        help="lvr2_reconstruct's -v/--voxelsize: smaller is finer but slower (default: %(default)s)",
    )
    parser.add_argument(
        "--point-cloud-manager",
        default="LVR2",
        choices=["FLANN", "PCL", "LVR2", "LBVH_CUDA"],
        help="lvr2_reconstruct's -p/--pcm (default: %(default)s)",
    )
    parser.add_argument(
        "--publish-topic",
        default="/move_base_flex/mesh_update",
        help="Topic to publish the reconstructed mesh on, for mesh_map's mesh_geometry_sub "
        "(default: %(default)s)",
    )
    parser.add_argument(
        "--frame-id",
        default="odom",
        help="Frame the mesh is published in; must match mesh_map's global_frame "
        "(default: %(default)s)",
    )
    parser.add_argument(
        "--publish-hold",
        type=float,
        default=2.0,
        help="Seconds to keep the node alive after publishing, so the message actually "
        "gets sent before this process exits (default: %(default)s)",
    )
    parser.add_argument(
        "--no-publish",
        action="store_true",
        help="Only write --output; skip publishing --publish-topic",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.0,
        help="Seconds between refresh cycles. 0 (default) captures/reconstructs/publishes "
        "once and exits; a positive value repeats indefinitely (Ctrl+C to stop), so the "
        "mesh keeps tracking FAST-LIO's growing map (default: %(default)s)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = _MapToMesh(args.topic, args.publish_topic)
    try:
        if args.interval <= 0:
            run_cycle(node, args)
        else:
            print(f"Refreshing every {args.interval}s. Press Ctrl+C to stop.")
            while True:
                try:
                    run_cycle(node, args)
                except (TimeoutError, ValueError, subprocess.CalledProcessError) as e:
                    print(f"Refresh cycle failed, will retry next interval: {e}", file=sys.stderr)
                deadline = time.time() + args.interval
                while time.time() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.5)
    except TimeoutError as e:
        # Only reached in single-shot mode -- the repeating branch above catches its own.
        sys.exit(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"Done. Point navigation.launch.py's mesh_map_path at {args.output}")


def run_cycle(node, args):
    with tempfile.NamedTemporaryFile(suffix=".ply", delete=False) as tmp:
        ply_path = tmp.name
    try:
        print(f"Waiting up to {args.timeout}s for a map on {args.topic}...")
        cloud_msg = capture_map_cloud(node, args.timeout)
        print(f"Captured map, writing -> {ply_path}")
        write_ply(cloud_msg, ply_path)
        print(f"Reconstructing mesh -> {args.output}")
        reconstruct_mesh(
            ply_path, args.output, args.voxel_size, args.point_cloud_manager
        )

        if not args.no_publish:
            print(f"Reading back {args.output} and publishing -> {args.publish_topic}")
            mesh_msg = mesh_file_to_geometry_msg(
                args.output, args.frame_id, node.get_clock().now().to_msg()
            )
            publish_mesh_update(node, mesh_msg, args.publish_hold)
    finally:
        os.unlink(ply_path)


if __name__ == "__main__":
    main()
