"""Drift detector node.

Subscribes to filtered odometry and plays drifting.mp3 via the USB speaker
when the robot is simultaneously moving forward and rotating (i.e. drifting).
A configurable cooldown prevents the sound from repeating too frequently.
"""

import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class DriftDetectorNode(Node):
    def __init__(self):
        super().__init__("drift_detector_node")

        self.declare_parameter("min_linear_velocity", 0.1)
        self.declare_parameter("min_angular_velocity", 0.3)
        self.declare_parameter("cooldown_seconds", 30.0)
        self.declare_parameter("audio_player", "mpg123")

        pkg_share = get_package_share_directory("perseus_lite_drifting")
        self._audio_path = f"{pkg_share}/audio/drifting.mp3"
        self._last_play_time = 0.0
        self._player_process = None

        self._odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self._odom_cb, 10
        )

        self.get_logger().info(
            f"Drift detector started — "
            f"linear >= {self.get_parameter('min_linear_velocity').value} m/s, "
            f"angular >= {self.get_parameter('min_angular_velocity').value} rad/s, "
            f"cooldown {self.get_parameter('cooldown_seconds').value}s"
        )

    def _is_drifting(self, msg: Odometry) -> bool:
        linear_speed = abs(msg.twist.twist.linear.x)
        angular_speed = abs(msg.twist.twist.angular.z)
        min_linear = self.get_parameter("min_linear_velocity").value
        min_angular = self.get_parameter("min_angular_velocity").value
        return linear_speed >= min_linear and angular_speed >= min_angular

    def _can_play(self) -> bool:
        if self._player_process is not None and self._player_process.poll() is None:
            return False
        cooldown = self.get_parameter("cooldown_seconds").value
        return (time.monotonic() - self._last_play_time) >= cooldown

    def _play_audio(self):
        player = self.get_parameter("audio_player").value
        try:
            self._player_process = subprocess.Popen(
                [player, "-q", self._audio_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._last_play_time = time.monotonic()
            self.get_logger().info("DRIFTING! Playing audio")
        except FileNotFoundError:
            self.get_logger().error(
                f"Audio player '{player}' not found. "
                f"Install with: sudo apt-get install {player}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to play audio: {e}")

    def _odom_cb(self, msg: Odometry):
        if self._is_drifting(msg) and self._can_play():
            self._play_audio()

    def destroy_node(self):
        if self._player_process is not None and self._player_process.poll() is None:
            self._player_process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriftDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
