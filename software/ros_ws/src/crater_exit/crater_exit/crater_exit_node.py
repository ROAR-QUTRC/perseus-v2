"""
Crater Exit Node — detects when the rover is stuck inside a crater
(surrounded by obstacles in the 2D scan) and drives forward slowly
until the IMU pitch indicates the rover has crested the rim.

Publishes velocity commands on a dedicated twist_mux topic at higher
priority than Nav2, bypassing the collision monitor.

On crater detection: cancels active Nav2 goals via the waypoints bridge.
On crater exit: re-sends remaining waypoints to resume navigation.
"""

import math
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from std_msgs.msg import String

from perseus_interfaces.msg import NavigationData, Waypoint
from perseus_interfaces.srv import RunWaypoints


class State(Enum):
    MONITORING = auto()
    CRATER_DETECTED = auto()
    EXITING = auto()
    CREST_DETECTED = auto()


def quaternion_to_pitch(q) -> float:
    """Extract pitch (rotation about Y axis) from quaternion, in degrees."""
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return math.degrees(pitch)


def quaternion_to_yaw(q) -> float:
    """Extract yaw from quaternion, in radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CraterExitNode(Node):
    def __init__(self):
        super().__init__("crater_exit_node")

        # Declare parameters
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/livox/imu/corrected")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel_crater_exit")
        self.declare_parameter("obstacle_distance_threshold", 1.5)
        self.declare_parameter("obstacle_coverage_threshold", 0.7)
        self.declare_parameter("detection_confirmation_count", 5)
        self.declare_parameter("min_quadrants_with_obstacles", 3)
        self.declare_parameter("exit_linear_speed", 0.08)
        self.declare_parameter("exit_timeout", 15.0)
        self.declare_parameter("pitch_threshold_deg", -3.0)
        self.declare_parameter("pitch_window_duration", 3.0)
        self.declare_parameter("enable_crater_exit", True)

        # Read parameters
        self._scan_topic = self.get_parameter("scan_topic").value
        self._imu_topic = self.get_parameter("imu_topic").value
        self._cmd_vel_topic = self.get_parameter("cmd_vel_out_topic").value
        self._obstacle_dist = self.get_parameter("obstacle_distance_threshold").value
        self._coverage_thresh = self.get_parameter("obstacle_coverage_threshold").value
        self._confirm_count = self.get_parameter("detection_confirmation_count").value
        self._min_quadrants = self.get_parameter("min_quadrants_with_obstacles").value
        self._exit_speed = self.get_parameter("exit_linear_speed").value
        self._exit_timeout = self.get_parameter("exit_timeout").value
        self._pitch_thresh = self.get_parameter("pitch_threshold_deg").value
        self._pitch_window = self.get_parameter("pitch_window_duration").value
        self._enabled = self.get_parameter("enable_crater_exit").value

        # State
        self._state = State.MONITORING
        self._crater_confirm_counter = 0
        self._exit_start_time = None
        self._latest_pitch_deg = 0.0
        self._baseline_pitch_deg = 0.0

        # Rolling pitch buffer: stores (timestamp_sec, pitch_deg) tuples
        self._pitch_buffer = deque()

        # Nav2 waypoint tracking
        self._cached_poses = []  # List of geometry_msgs/Pose from the bridge
        self._poses_remaining = 0
        self._nav_active = False

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Sensor subscriptions
        self._scan_sub = self.create_subscription(
            LaserScan, self._scan_topic, self._scan_callback, sensor_qos
        )
        self._imu_sub = self.create_subscription(
            Imu, self._imu_topic, self._imu_callback, sensor_qos
        )

        # Waypoint cache subscription (published by nav2_waypoints_bridge)
        self._waypoints_sub = self.create_subscription(
            PoseArray, "/autonomy/active_waypoints", self._waypoints_callback, latched_qos
        )

        # Navigation feedback subscription (published by nav2_waypoints_bridge)
        self._nav_info_sub = self.create_subscription(
            NavigationData, "/autonomy/navigation_info", self._nav_info_callback, 10
        )

        # Service clients for the waypoints bridge
        self._cancel_client = self.create_client(
            RunWaypoints, "/autonomy/cancel_waypoints"
        )
        self._run_client = self.create_client(
            RunWaypoints, "/autonomy/run_waypoints"
        )

        # Publishers
        self._cmd_vel_pub = self.create_publisher(TwistStamped, self._cmd_vel_topic, 10)
        self._status_pub = self.create_publisher(String, "/crater_exit/status", 10)

        # Timer for exit velocity publishing (20 Hz when active)
        self._cmd_timer = self.create_timer(0.05, self._cmd_timer_callback)

        self.get_logger().info(
            f"Crater exit node started — scan: {self._scan_topic}, "
            f"imu: {self._imu_topic}, cmd_vel: {self._cmd_vel_topic}"
        )
        self._publish_status()

    # ── Waypoint tracking ─────────────────────────────────────────────

    def _waypoints_callback(self, msg: PoseArray):
        """Cache waypoints published by the bridge when a goal is sent."""
        self._cached_poses = list(msg.poses)
        self.get_logger().info(f"Cached {len(self._cached_poses)} waypoints")

    def _nav_info_callback(self, msg: NavigationData):
        """Track navigation progress from the bridge."""
        self._poses_remaining = msg.number_of_poses_remaining
        self._nav_active = msg.navigation_active

    # ── Sensor callbacks ──────────────────────────────────────────────

    def _scan_callback(self, msg: LaserScan):
        if not self._enabled:
            return

        if self._state == State.MONITORING:
            if self._is_crater_signature(msg):
                self._crater_confirm_counter += 1
                if self._crater_confirm_counter >= self._confirm_count:
                    self._transition_to(State.CRATER_DETECTED)
            else:
                self._crater_confirm_counter = 0

        elif self._state == State.EXITING:
            # If scan clears, we may have exited even without a pitch trigger
            if not self._is_crater_signature(msg):
                self._crater_confirm_counter += 1
                if self._crater_confirm_counter >= self._confirm_count:
                    self.get_logger().info("Scan cleared — crater exit confirmed via scan")
                    self._transition_to(State.CREST_DETECTED)
            else:
                self._crater_confirm_counter = 0

    def _is_crater_signature(self, scan: LaserScan) -> bool:
        """Check if the scan shows obstacles surrounding the robot."""
        total_rays = len(scan.ranges)
        if total_rays == 0:
            return False

        valid_count = 0
        close_count = 0
        quadrant_has_obstacle = [False, False, False, False]

        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max or math.isinf(r) or math.isnan(r):
                continue
            valid_count += 1
            if r < self._obstacle_dist:
                close_count += 1
                quadrant = min((i * 4) // total_rays, 3)
                quadrant_has_obstacle[quadrant] = True

        if valid_count == 0:
            return False

        coverage = close_count / valid_count
        num_quadrants = sum(quadrant_has_obstacle)

        return coverage >= self._coverage_thresh and num_quadrants >= self._min_quadrants

    def _imu_callback(self, msg: Imu):
        self._latest_pitch_deg = quaternion_to_pitch(msg.orientation)
        now_sec = self.get_clock().now().nanoseconds / 1e9

        if self._state == State.EXITING:
            # Add sample to rolling buffer
            self._pitch_buffer.append((now_sec, self._latest_pitch_deg))

            # Evict samples older than the window
            cutoff = now_sec - self._pitch_window
            while self._pitch_buffer and self._pitch_buffer[0][0] < cutoff:
                self._pitch_buffer.popleft()

            # Only evaluate once the buffer spans the full window duration
            if len(self._pitch_buffer) >= 2:
                oldest_time = self._pitch_buffer[0][0]
                window_filled = (now_sec - oldest_time) >= self._pitch_window

                if window_filled:
                    avg_pitch = sum(p for _, p in self._pitch_buffer) / len(self._pitch_buffer)
                    if avg_pitch < self._pitch_thresh:
                        self.get_logger().info(
                            f"Avg pitch {avg_pitch:.1f}° over {self._pitch_window}s "
                            f"< {self._pitch_thresh}° — crater rim crested"
                        )
                        self._transition_to(State.CREST_DETECTED)

    # ── Timer ─────────────────────────────────────────────────────────

    def _cmd_timer_callback(self):
        if self._state == State.EXITING:
            # Check timeout
            now = self.get_clock().now()
            elapsed = (now - self._exit_start_time).nanoseconds / 1e9
            if elapsed > self._exit_timeout:
                self.get_logger().warn(
                    f"Crater exit timed out after {elapsed:.1f}s — aborting"
                )
                self._transition_to(State.MONITORING)
                return

            # Publish forward velocity
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.x = self._exit_speed
            self._cmd_vel_pub.publish(cmd)

    # ── State transitions ─────────────────────────────────────────────

    def _transition_to(self, new_state: State):
        old_state = self._state
        self._state = new_state
        self.get_logger().info(f"State: {old_state.name} -> {new_state.name}")

        if new_state == State.CRATER_DETECTED:
            self._baseline_pitch_deg = self._latest_pitch_deg
            self.get_logger().info(
                f"Crater detected — baseline pitch: {self._baseline_pitch_deg:.1f}°, "
                f"beginning exit at {self._exit_speed} m/s"
            )

            # Cancel active Nav2 goals so the BT doesn't fail messily
            self._cancel_nav2_goals()

            # Go directly to exiting
            self._state = State.EXITING
            self._exit_start_time = self.get_clock().now()
            self._pitch_buffer.clear()
            self._crater_confirm_counter = 0

        elif new_state == State.CREST_DETECTED:
            self._stop_robot()
            self.get_logger().info("Crater exit complete — re-sending remaining waypoints")

            # Re-send remaining waypoints to resume navigation
            self._resend_remaining_waypoints()

            self._state = State.MONITORING
            self._crater_confirm_counter = 0
            self._pitch_buffer.clear()

        elif new_state == State.MONITORING:
            self._stop_robot()
            self._crater_confirm_counter = 0
            self._pitch_buffer.clear()
            self._exit_start_time = None

        self._publish_status()

    # ── Nav2 integration ──────────────────────────────────────────────

    def _cancel_nav2_goals(self):
        """Cancel active Nav2 goals via the waypoints bridge."""
        if not self._cancel_client.service_is_ready():
            self.get_logger().warn("Cancel service not available — skipping cancel")
            return

        req = RunWaypoints.Request()
        req.waypoints = []  # Cancel doesn't need waypoints
        future = self._cancel_client.call_async(req)
        future.add_done_callback(self._on_cancel_done)

    def _on_cancel_done(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f"Nav2 cancel: {resp.message}")
        except Exception as e:
            self.get_logger().warn(f"Nav2 cancel failed: {e}")

    def _resend_remaining_waypoints(self):
        """Re-send remaining waypoints to Nav2 via the waypoints bridge."""
        if not self._cached_poses:
            self.get_logger().warn("No cached waypoints — cannot resume navigation")
            return

        if self._poses_remaining <= 0:
            self.get_logger().info("No remaining waypoints to re-send")
            return

        # Get the last N poses (the ones not yet completed)
        remaining_poses = self._cached_poses[-self._poses_remaining :]

        if not self._run_client.service_is_ready():
            self.get_logger().warn("Run service not available — cannot resume navigation")
            return

        # Convert Pose objects back to Waypoint messages
        req = RunWaypoints.Request()
        for pose in remaining_poses:
            wp = Waypoint()
            wp.name = ""
            wp.x = pose.position.x
            wp.y = pose.position.y
            wp.yaw = quaternion_to_yaw(pose.orientation)
            req.waypoints.append(wp)

        self.get_logger().info(f"Re-sending {len(req.waypoints)} remaining waypoints")
        future = self._run_client.call_async(req)
        future.add_done_callback(self._on_resend_done)

    def _on_resend_done(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"Waypoints resumed: {resp.message}")
            else:
                self.get_logger().warn(f"Waypoint resume failed: {resp.message}")
        except Exception as e:
            self.get_logger().warn(f"Waypoint resume call failed: {e}")

    # ── Helpers ────────────────────────────────────────────────────────

    def _stop_robot(self):
        """Publish zero velocity to stop the robot."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        self._cmd_vel_pub.publish(cmd)

    def _publish_status(self):
        msg = String()
        msg.data = self._state.name
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CraterExitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
