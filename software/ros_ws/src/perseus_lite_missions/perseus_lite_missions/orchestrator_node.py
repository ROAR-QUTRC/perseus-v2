"""
Mission orchestrator — owns the segment state machine.

Reads the mission descriptor (mission.yaml), tracks which segment the viewer
is in, and publishes mission state on /mission/state for the web UI to render.

When a challenge passes, kills the relevant node(s) and relaunches them with
the reference params. Uses subprocess + ros2 CLI rather than lifecycle
reconfigure because slam_toolbox's lifecycle support is unreliable for
param-swap restarts.
"""

import json
from pathlib import Path
import subprocess
import threading

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

import yaml


def find_segment_index(segments: list, target_id: int) -> int | None:
    """Return the index of the segment with the given id, or None."""
    for idx, seg in enumerate(segments):
        if seg.get("id") == target_id:
            return idx
    return None


def merge_unlocked_badges(existing, badge_ids: list[str]) -> dict:
    """Merge badge ids into a badges document, preserving prior unlocks."""
    if not isinstance(existing, dict):
        existing = {}
    existing.setdefault("unlocked", [])
    for bid in badge_ids:
        if bid not in existing["unlocked"]:
            existing["unlocked"].append(bid)
    return existing


class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__("mission_orchestrator")

        self.declare_parameter("mission_yaml", "")
        self.declare_parameter("use_sim_time", True)

        mission_path = Path(self.get_parameter("mission_yaml").value)
        if not mission_path.is_file():
            self.get_logger().error(
                f"Mission descriptor not found at {mission_path}; orchestrator idle."
            )
            self.mission = None
        else:
            with mission_path.open() as f:
                self.mission = yaml.safe_load(f)
            self.get_logger().info(
                f"Loaded mission {self.mission.get('id')!r} "
                f"({len(self.mission.get('segments', []))} segments)."
            )

        self._missions_share = get_package_share_directory("perseus_lite_missions")
        self._challenge_1_done = False
        self._challenge_2_done = False
        self._slam_proc: subprocess.Popen | None = None
        self._slam_watchdog = None

        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.badge_pub = self.create_publisher(String, "/mission/badge", 10)
        self.current_segment_idx = 0

        self.create_subscription(
            Bool, "/mission/challenge_1/passed", self._on_challenge_1, 10
        )
        self.create_subscription(
            Bool, "/mission/challenge_2/passed", self._on_challenge_2, 10
        )

        self.create_timer(1.0, self._publish_state)

    def _publish_state(self):
        if self.mission is None:
            return
        segments = self.mission.get("segments", [])
        if self.current_segment_idx >= len(segments):
            return
        seg = segments[self.current_segment_idx]
        msg = String()
        msg.data = json.dumps({
            "segment_idx": self.current_segment_idx,
            "segment_id": seg.get("id"),
            "segment_kind": seg.get("kind"),
            "segment_title": seg.get("title"),
            "challenge_1_done": self._challenge_1_done,
            "challenge_2_done": self._challenge_2_done,
        })
        self.state_pub.publish(msg)

    def _on_challenge_1(self, msg: Bool):
        if not msg.data or self._challenge_1_done:
            return
        self._challenge_1_done = True
        self.get_logger().info(
            "Challenge 1 passed — restarting slam_toolbox with reference params."
        )
        self._advance_to_segment(5)
        self._emit_badge("mapping-tier-1")
        threading.Thread(target=self._relaunch_slam, daemon=True).start()

    def _on_challenge_2(self, msg: Bool):
        if not msg.data or self._challenge_2_done:
            return
        self._challenge_2_done = True
        self.get_logger().info("Challenge 2 passed — mission complete.")
        self._advance_to_segment(8)
        self._emit_badge("navigation-tier-1")
        self._persist_badges()

    def _advance_to_segment(self, target_id: int):
        if self.mission is None:
            return
        segments = self.mission.get("segments", [])
        idx = find_segment_index(segments, target_id)
        if idx is None:
            self.get_logger().warning(f"No segment with id {target_id} in mission.")
            return
        self.current_segment_idx = idx
        self.get_logger().info(
            f"Advanced to segment {target_id}: {segments[idx].get('title')}"
        )

    def _emit_badge(self, badge_id: str):
        if self.mission is None:
            return
        badges = self.mission.get("badges", [])
        for badge in badges:
            if badge.get("id") == badge_id:
                msg = String()
                msg.data = json.dumps(badge)
                self.badge_pub.publish(msg)
                self.get_logger().info(f"Badge unlocked: {badge.get('title')}")
                return

    def _relaunch_slam(self):
        for cmd in (
            ["ros2", "lifecycle", "set", "/slam_toolbox", "shutdown"],
            ["pkill", "-f", "async_slam_toolbox_node"],
        ):
            try:
                subprocess.run(cmd, capture_output=True, timeout=5)
            except (subprocess.TimeoutExpired, OSError) as e:
                self.get_logger().warning(f"{cmd[0]} step failed ({e}); continuing.")

        ref_params = str(
            Path(self._missions_share)
            / "missions"
            / "m00-cold-open"
            / "configs"
            / "slam_params.reference.yaml"
        )
        use_sim_time = self.get_parameter("use_sim_time").value
        self.get_logger().info(f"Relaunching slam_toolbox with {ref_params}")
        try:
            self._slam_proc = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "slam_toolbox",
                    "async_slam_toolbox_node",
                    "--ros-args",
                    "--params-file",
                    ref_params,
                    "-p",
                    f"use_sim_time:={'true' if use_sim_time else 'false'}",
                ],
                stdin=subprocess.DEVNULL,
            )
        except OSError as e:
            self.get_logger().error(f"Failed to relaunch slam_toolbox: {e}")
            return
        self._slam_watchdog = self.create_timer(2.0, self._check_slam_proc)

    def _check_slam_proc(self):
        if self._slam_proc is None:
            return
        code = self._slam_proc.poll()
        if code is not None:
            self.get_logger().error(
                f"Relaunched slam_toolbox exited with code {code}; "
                "mapping is no longer running."
            )
            self._slam_watchdog.cancel()

    def shutdown(self):
        """Stop the relaunched slam_toolbox process, if any."""
        if self._slam_proc and self._slam_proc.poll() is None:
            self._slam_proc.terminate()
            try:
                self._slam_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._slam_proc.kill()

    def _persist_badges(self):
        badge_dir = Path.home() / ".config" / "rover-robotics"
        badge_dir.mkdir(parents=True, exist_ok=True)
        badge_file = badge_dir / "badges.json"
        existing = {}
        if badge_file.is_file():
            try:
                existing = json.loads(badge_file.read_text())
            except (json.JSONDecodeError, OSError):
                pass
        existing = merge_unlocked_badges(
            existing, ["mapping-tier-1", "navigation-tier-1"]
        )
        badge_file.write_text(json.dumps(existing, indent=2) + "\n")
        self.get_logger().info(f"Badges persisted to {badge_file}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
