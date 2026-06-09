"""
Challenge 1 checker — watches the slam_params YAML on disk.

The simplest viable gate: pass when the user has saved a slam_params file
with `resolution <= 0.1 m` and `max_laser_range >= 8 m`. The thresholds are
loose enough that any reasonable answer (the script suggests 0.05 and 12.0)
passes, but the obviously-broken starting values (0.30 and 3.0) do not.

A future iteration can replace the YAML-watch with a real /map quality
analysis (e.g. wall-pixel density along a known corridor segment), but the
YAML check is robust enough to ship Mission Zero on.
"""

import os
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

import yaml


def evaluate_slam_params(
    doc: dict, min_resolution_inverse: float, min_max_laser_range: float
) -> bool:
    """Check a parsed slam_toolbox params document against the thresholds."""
    params = (doc or {}).get("slam_toolbox", {}).get("ros__parameters", {})
    resolution = params.get("resolution")
    max_range = params.get("max_laser_range")
    if resolution is None or max_range is None:
        return False
    return (
        resolution > 0.0
        and (1.0 / resolution) >= min_resolution_inverse
        and max_range >= min_max_laser_range
    )


class MapQualityChecker(Node):
    def __init__(self):
        super().__init__("map_quality_checker")

        self.declare_parameter("slam_params_file", "")
        self.declare_parameter("min_resolution_inverse", 10.0)  # resolution <= 0.1
        self.declare_parameter("min_max_laser_range", 8.0)

        self.params_path = Path(self.get_parameter("slam_params_file").value)
        self.min_resolution_inverse = (
            self.get_parameter("min_resolution_inverse").value
        )
        self.min_max_laser_range = self.get_parameter("min_max_laser_range").value

        self.passed = False
        self.last_mtime = 0.0
        self.pub = self.create_publisher(Bool, "/mission/challenge_1/passed", 10)
        self.create_timer(1.0, self.tick)

        if not self.params_path.is_file():
            self.get_logger().warning(
                f"slam params file not found at {self.params_path}; "
                "checker will idle until it appears."
            )

    def tick(self):
        if self.passed:
            self.pub.publish(Bool(data=True))
            return
        if not self.params_path.is_file():
            return
        try:
            mtime = os.path.getmtime(self.params_path)
        except OSError:
            return
        if mtime == self.last_mtime:
            return
        self.last_mtime = mtime
        if self.evaluate():
            self.passed = True
            self.get_logger().info("Challenge 1 passed — map params look right.")

    def evaluate(self) -> bool:
        try:
            with self.params_path.open() as f:
                doc = yaml.safe_load(f) or {}
        except yaml.YAMLError as e:
            self.get_logger().info(f"slam params YAML not yet valid: {e}")
            return False
        return evaluate_slam_params(
            doc, self.min_resolution_inverse, self.min_max_laser_range
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapQualityChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
