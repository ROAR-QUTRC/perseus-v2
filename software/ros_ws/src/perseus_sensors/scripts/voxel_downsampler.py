#!/usr/bin/env python3
"""Voxel-grid downsampling node for the Livox point cloud.

Subscribes to a raw sensor_msgs/PointCloud2 topic (default: /livox/lidar) and republishes
a version with fewer points, by keeping at most one point per small cube ("voxel") of space.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class VoxelDownsampler(Node):
    def __init__(self):
        # "voxel_downsampler" is the node's name, as it will show up in `ros2 node list`.
        super().__init__("voxel_downsampler")

        # declare_parameter registers a ROS2 parameter with a default value. You can override
        # any of these from a launch file, a yaml config, or `ros2 run ... --ros-args -p key:=value`
        # without touching this script.
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar/downsampled")
        self.declare_parameter("voxel_size_m", 0.1)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.voxel_size_m = self.get_parameter("voxel_size_m").value

        # A subscriber: whenever a message arrives on input_topic, ROS2 calls
        # self._point_cloud_callback with it. "10" is the queue depth (how many messages to
        # buffer if callbacks fall behind).
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._point_cloud_callback, 10
        )

        # A publisher: lets us push new messages out on output_topic.
        self._publisher = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(
            f"Voxel downsampler: {input_topic} -> {output_topic} "
            f"(voxel size {self.voxel_size_m} m)"
        )

    def _point_cloud_callback(self, msg: PointCloud2):
        # read_points turns the raw PointCloud2 byte buffer into an iterable of point tuples,
        # e.g. (x, y, z, intensity, tag, line, timestamp) -- whatever fields the message has.
        # We keep every field, not just x/y/z, so nothing the Livox driver adds gets dropped.
        field_names = [f.name for f in msg.fields]
        points = list(point_cloud2.read_points(msg, field_names=field_names, skip_nans=True))

        seen_voxels = set()
        kept_points = []
        for point in points:
            x, y, z = point[0], point[1], point[2]

            # Some sim point clouds include infinite coordinates for "no return" rays, which
            # skip_nans doesn't catch. Drop those before dividing by voxel_size_m.
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            # Snap this point's (x, y, z) to an integer grid coordinate. Points that land in
            # the same cube share the same (vx, vy, vz) key.
            vx = math.floor(x / self.voxel_size_m)
            vy = math.floor(y / self.voxel_size_m)
            vz = math.floor(z / self.voxel_size_m)
            voxel_key = (vx, vy, vz)

            # Keep only the first point we see for each voxel; skip every later point that
            # falls in the same cube. That's the entire downsampling logic.
            if voxel_key not in seen_voxels:
                seen_voxels.add(voxel_key)
                kept_points.append(point)

        # Rebuild a PointCloud2 message from the surviving points, using the same field layout
        # as the input so downstream nodes (FAST-LIO, etc.) see the fields they expect.
        downsampled_msg = point_cloud2.create_cloud(msg.header, msg.fields, kept_points)
        self._publisher.publish(downsampled_msg)

        self.get_logger().info(
            f"Downsampled {len(points)} -> {len(kept_points)} points",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = VoxelDownsampler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
