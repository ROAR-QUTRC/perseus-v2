"""
Challenge 2 checker — passes when the rover reaches the leak pose.

Stub implementation: reads the leak coords from mission.yaml and subscribes
to /tf to compute base_footprint distance from the leak. Passes when the
rover is within `tolerance_m` of the leak. The real version will also gate
on collision count (no contacts during the run) and on NavigateToPose
action status.
"""

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Bool

import tf2_ros

import yaml


class GoalReachedChecker(Node):
    def __init__(self):
        super().__init__("goal_reached_checker")

        self.declare_parameter("mission_yaml", "")
        self.declare_parameter("tolerance_m", 0.5)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")

        mission_path = Path(self.get_parameter("mission_yaml").value)
        self.tolerance_m = self.get_parameter("tolerance_m").value
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value

        self.leak_xy = None
        if mission_path.is_file():
            with mission_path.open() as f:
                mission = yaml.safe_load(f) or {}
            leak = mission.get("poses", {}).get("leak", {})
            if "x" in leak and "y" in leak:
                self.leak_xy = (float(leak["x"]), float(leak["y"]))
                self.get_logger().info(
                    f"Leak target: ({self.leak_xy[0]:.2f}, {self.leak_xy[1]:.2f}) "
                    f"tolerance {self.tolerance_m:.2f} m"
                )
        if self.leak_xy is None:
            self.get_logger().warning("No leak pose configured; checker will idle.")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.passed = False
        self.pub = self.create_publisher(Bool, "/mission/challenge_2/passed", 10)
        self.create_timer(0.5, self.tick)

    def tick(self):
        if self.passed:
            self.pub.publish(Bool(data=True))
            return
        if self.leak_xy is None:
            return
        try:
            tx = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ):
            return
        dx = tx.transform.translation.x - self.leak_xy[0]
        dy = tx.transform.translation.y - self.leak_xy[1]
        if math.hypot(dx, dy) <= self.tolerance_m:
            self.passed = True
            self.get_logger().info("Challenge 2 passed — rover at the leak.")


def main(args=None):
    rclpy.init(args=args)
    node = GoalReachedChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
