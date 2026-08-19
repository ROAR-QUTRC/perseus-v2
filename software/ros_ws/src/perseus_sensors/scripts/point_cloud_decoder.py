#!/usr/bin/env python3
"""Decoder node: Draco-decompresses a CompressedPointCloud back into a PointCloud2.

Subscribes to a perseus_interfaces/CompressedPointCloud topic (default:
/livox/lidar/downsampled), Draco-decodes the x/y/z positions, and republishes a
sensor_msgs/PointCloud2 for visualization (e.g. RViz). Intended to run only where a decoded
cloud is actually needed -- e.g. a base-station receiver -- since decoding undoes the
bandwidth saving that compression bought on the wire between rover and receiver.
"""

import DracoPy
import numpy as np
import rclpy
from perseus_interfaces.msg import CompressedPointCloud
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudDecoder(Node):
    def __init__(self):
        super().__init__("point_cloud_decoder")

        self.declare_parameter("input_topic", "/livox/lidar/downsampled")
        self.declare_parameter("output_topic", "/livox/lidar/downsampled/points")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._subscription = self.create_subscription(
            CompressedPointCloud, input_topic, self._compressed_point_cloud_callback, 10
        )
        self._publisher = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(f"Point cloud decoder: {input_topic} -> {output_topic}")

    def _compressed_point_cloud_callback(self, msg: CompressedPointCloud):
        decoded = DracoPy.decode(bytes(msg.draco_data))
        positions = np.asarray(decoded.points, dtype=np.float32)
        point_count = positions.shape[0]

        # timestamps/tags/lines travel uncompressed, in the same per-point order as the
        # Draco-encoded positions (see voxel_downsampler.py), so we can zip them back
        # together directly. They may be empty if the source cloud didn't have those
        # fields (e.g. simulated Livox data), in which case we publish x/y/z only.
        has_side_channels = (
            len(msg.timestamps) == point_count
            and len(msg.tags) == point_count
            and len(msg.lines) == point_count
        )

        fields = [
            point_cloud2.PointField(
                name="x", offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1
            ),
            point_cloud2.PointField(
                name="y", offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1
            ),
            point_cloud2.PointField(
                name="z", offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1
            ),
        ]

        if has_side_channels:
            fields += [
                point_cloud2.PointField(
                    name="timestamp",
                    offset=12,
                    datatype=point_cloud2.PointField.FLOAT64,
                    count=1,
                ),
                point_cloud2.PointField(
                    name="tag",
                    offset=20,
                    datatype=point_cloud2.PointField.UINT8,
                    count=1,
                ),
                point_cloud2.PointField(
                    name="line",
                    offset=21,
                    datatype=point_cloud2.PointField.UINT8,
                    count=1,
                ),
            ]
            points = [
                (
                    float(positions[i, 0]),
                    float(positions[i, 1]),
                    float(positions[i, 2]),
                    float(msg.timestamps[i]),
                    int(msg.tags[i]),
                    int(msg.lines[i]),
                )
                for i in range(point_count)
            ]
        else:
            points = [
                (float(positions[i, 0]), float(positions[i, 1]), float(positions[i, 2]))
                for i in range(point_count)
            ]

        output_msg = point_cloud2.create_cloud(msg.header, fields, points)
        self._publisher.publish(output_msg)

        self.get_logger().info(
            f"Decoded {point_count} points from {len(msg.draco_data)} Draco bytes"
            f"{'' if has_side_channels else ' (no timestamp/tag/line)'}",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDecoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
