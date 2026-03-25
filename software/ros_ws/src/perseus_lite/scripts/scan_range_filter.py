#!/usr/bin/env python3
"""Minimal LaserScan range filter node.

Subscribes to /scan, sets readings below min_range to inf (invalid),
updates range_min in the message, and republishes to /scan_filtered.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanRangeFilter(Node):
    def __init__(self):
        super().__init__("scan_range_filter")
        self.declare_parameter("min_range", 0.5)
        self._min_range = self.get_parameter("min_range").value

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(LaserScan, "/scan_filtered", qos)
        self._sub = self.create_subscription(LaserScan, "/scan", self._cb, qos)
        self.get_logger().info(f"Filtering /scan -> /scan_filtered (min_range={self._min_range}m)")

    def _cb(self, msg: LaserScan):
        inf = float("inf")
        min_r = self._min_range
        msg.ranges = [r if r >= min_r else inf for r in msg.ranges]
        msg.range_min = min_r
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScanRangeFilter())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
