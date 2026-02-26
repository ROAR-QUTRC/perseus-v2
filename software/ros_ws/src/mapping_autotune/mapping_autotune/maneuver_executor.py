"""Drives the robot through repeatable test maneuvers for SLAM quality evaluation."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class ManeuverExecutor:
    """Closed-loop maneuver executor that drives repeatable motion patterns.

    Uses odometry feedback to drive precise distances and rotations,
    publishing velocity commands on /joy_vel (TwistStamped).
    """

    def __init__(self, node):
        """Initialize the maneuver executor.

        Args:
            node: An rclpy.node.Node instance (the autotune_node).
                  Must already be spinning in a separate thread.
        """
        self._node = node

        # Publisher for velocity commands
        self._cmd_pub = self._node.create_publisher(TwistStamped, "/joy_vel", 10)

        # Subscriber for odometry feedback
        self._odom_sub = self._node.create_subscription(
            Odometry,
            "/diff_drive_base_controller/odom",
            self._odom_callback,
            10,
        )

        # Current pose state from odometry
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0
        self._current_vx = 0.0
        self._current_vyaw = 0.0
        self._odom_received = False

        # Control parameters
        self._publish_rate = 20.0  # Hz
        self._position_tolerance = 0.05  # meters
        self._angle_tolerance = 0.05  # radians

        # Logging state
        self._logging_active = False
        self._odom_log = []
        self._imu_log = []
        self._imu_sub = None

    # ── Odom callback ─────────────────────────────────────────────────

    def _odom_callback(self, msg):
        """Store the latest odometry pose and velocity."""
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Store velocities
        self._current_vx = msg.twist.twist.linear.x
        self._current_vyaw = msg.twist.twist.angular.z

        self._odom_received = True

        # Log if active
        if self._logging_active:
            self._odom_log.append(
                {
                    "timestamp": time.time(),
                    "x": self._current_x,
                    "y": self._current_y,
                    "yaw": self._current_yaw,
                    "vx": self._current_vx,
                    "vyaw": self._current_vyaw,
                }
            )

    # ── IMU callback ──────────────────────────────────────────────────

    def _imu_callback(self, msg):
        """Store IMU data when logging is active."""
        if self._logging_active:
            self._imu_log.append(
                {
                    "timestamp": time.time(),
                    "ax": msg.linear_acceleration.x,
                    "ay": msg.linear_acceleration.y,
                    "az": msg.linear_acceleration.z,
                    "gx": msg.angular_velocity.x,
                    "gy": msg.angular_velocity.y,
                    "gz": msg.angular_velocity.z,
                }
            )

    # ── Main entry point ──────────────────────────────────────────────

    def execute_maneuver(self, maneuver_params=None):
        """Execute a test maneuver pattern.

        Args:
            maneuver_params: Optional dict with keys:
                - linear_speed (float): Forward speed in m/s (default 0.2)
                - rotation_speed (float): Rotation speed in rad/s (default 0.5)
                - pattern (str): 'box_return' or 'corridor' (default 'box_return')

        Returns:
            True if maneuver completed successfully, False on timeout/error.
        """
        if maneuver_params is None:
            maneuver_params = {}

        linear_speed = maneuver_params.get("linear_speed", 0.2)
        rotation_speed = maneuver_params.get("rotation_speed", 0.5)
        pattern = maneuver_params.get("pattern", "box_return")

        dispatch = {
            "box_return": self._execute_box_return,
            "corridor": self._execute_corridor,
            "straight": self._execute_straight,
            "small_turn": self._execute_small_turn,
            "big_turn": self._execute_big_turn,
        }

        executor = dispatch.get(pattern, self._execute_box_return)
        return executor(linear_speed, rotation_speed)

    # ── Maneuver patterns ─────────────────────────────────────────────

    def _execute_box_return(self, linear_speed, rotation_speed):
        """Drive forward, 180-degree turn, drive back, 180-degree turn.

        Sequence:
            1. Drive forward 1.0m
            2. Pause 1.0s
            3. Rotate 180 degrees
            4. Pause 1.0s
            5. Drive forward 1.0m
            6. Pause 1.0s
            7. Rotate 180 degrees (return to start heading)

        Returns:
            True if all steps completed, False on any failure.
        """
        steps = [
            ("drive", 1.0, linear_speed),
            ("pause", 1.0, None),
            ("rotate", math.pi, rotation_speed),
            ("pause", 1.0, None),
            ("drive", 1.0, linear_speed),
            ("pause", 1.0, None),
            ("rotate", math.pi, rotation_speed),
        ]

        return self._run_step_sequence(steps)

    def _execute_corridor(self, linear_speed, rotation_speed):
        """Drive a rectangular corridor pattern.

        Sequence:
            1. Drive forward 2.0m
            2. Rotate 90 degrees
            3. Drive forward 1.0m
            4. Rotate 90 degrees
            5. Drive forward 2.0m
            6. Rotate 90 degrees
            7. Drive forward 1.0m
            8. Rotate 90 degrees (complete rectangle)

        Returns:
            True if all steps completed, False on any failure.
        """
        steps = [
            ("drive", 2.0, linear_speed),
            ("rotate", math.pi / 2.0, rotation_speed),
            ("drive", 1.0, linear_speed),
            ("rotate", math.pi / 2.0, rotation_speed),
            ("drive", 2.0, linear_speed),
            ("rotate", math.pi / 2.0, rotation_speed),
            ("drive", 1.0, linear_speed),
            ("rotate", math.pi / 2.0, rotation_speed),
        ]

        return self._run_step_sequence(steps)

    def _execute_straight(self, linear_speed, rotation_speed):
        """Drive forward, 180-degree turn, drive back to start, 180-degree turn.

        Simplest pattern — tests pure linear SLAM with no lateral movement.

        Sequence:
            1. Drive forward 1.0m
            2. Pause 1.0s
            3. Rotate 180 degrees
            4. Pause 1.0s
            5. Drive forward 1.0m (back to start)
            6. Pause 1.0s
            7. Rotate 180 degrees (return to start heading)

        Returns:
            True if all steps completed, False on any failure.
        """
        steps = [
            ("drive", 1.0, linear_speed),
            ("pause", 1.0, None),
            ("rotate", math.pi, rotation_speed),
            ("pause", 1.0, None),
            ("drive", 1.0, linear_speed),
            ("pause", 1.0, None),
            ("rotate", math.pi, rotation_speed),
        ]

        return self._run_step_sequence(steps)

    def _execute_small_turn(self, linear_speed, rotation_speed):
        """Drive forward, 45-degree turn, short drive, then reverse the path.

        Tests gentle rotation handling in SLAM.

        Sequence:
            1. Drive forward 1.0m
            2. Pause 1.0s
            3. Rotate 45 degrees
            4. Drive forward 0.5m
            5. Pause 1.0s
            6. Rotate -45 degrees (undo turn)
            7. Drive backward (forward 1.0m toward start after 180)
            8. Rotate 180 degrees

        Returns:
            True if all steps completed, False on any failure.
        """
        quarter_pi = math.pi / 4.0
        steps = [
            ("drive", 1.0, linear_speed),
            ("pause", 1.0, None),
            ("rotate", quarter_pi, rotation_speed),
            ("drive", 0.5, linear_speed),
            ("pause", 1.0, None),
            ("rotate", -quarter_pi, rotation_speed),
            ("pause", 0.5, None),
            ("drive", 1.0, linear_speed),
            ("pause", 1.0, None),
            ("rotate", math.pi, rotation_speed),
        ]

        return self._run_step_sequence(steps)

    def _execute_big_turn(self, linear_speed, rotation_speed):
        """Drive forward, 90-degree turn, drive, then reverse the path.

        Tests moderate rotation — between small_turn and box_return.

        Sequence:
            1. Drive forward 1.0m
            2. Pause 1.0s
            3. Rotate 90 degrees
            4. Drive forward 1.0m
            5. Pause 1.0s
            6. Rotate -90 degrees (undo turn)
            7. Drive forward 1.0m (back toward start)
            8. Rotate 180 degrees

        Returns:
            True if all steps completed, False on any failure.
        """
        half_pi = math.pi / 2.0
        steps = [
            ("drive", 1.0, linear_speed),
            ("pause", 1.0, None),
            ("rotate", half_pi, rotation_speed),
            ("drive", 1.0, linear_speed),
            ("pause", 1.0, None),
            ("rotate", -half_pi, rotation_speed),
            ("pause", 0.5, None),
            ("drive", 1.0, linear_speed),
            ("pause", 1.0, None),
            ("rotate", math.pi, rotation_speed),
        ]

        return self._run_step_sequence(steps)

    def _run_step_sequence(self, steps):
        """Execute a sequence of drive/rotate/pause steps.

        Args:
            steps: List of tuples (type, value, speed) where type is
                   'drive', 'rotate', or 'pause'.

        Returns:
            True if all steps completed successfully.
        """
        for step_type, value, speed in steps:
            if step_type == "drive":
                if not self._drive_distance(value, speed):
                    self._stop()
                    return False
            elif step_type == "rotate":
                if not self._rotate_angle(value, speed):
                    self._stop()
                    return False
            elif step_type == "pause":
                self._pause(value)

        self._stop()
        return True

    # ── Control methods ───────────────────────────────────────────────

    def _drive_distance(self, distance, speed):
        """Drive forward for distance/speed seconds at constant speed.

        Uses time-based control because the wheel odometry may not be
        accurately calibrated.  The robot drives at *speed* for
        ``distance / speed`` seconds, then ramps down and stops.

        Args:
            distance: Nominal distance in meters (used to compute drive time).
            speed: Forward speed in m/s.

        Returns:
            True when the timed drive completes.
        """
        drive_time = distance / speed
        period = 1.0 / self._publish_rate
        start_time = time.time()

        self._node.get_logger().info(
            f"Drive: speed={speed:.2f} m/s for {drive_time:.1f}s "
            f"(~{distance:.2f}m)"
        )

        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed >= drive_time:
                break
            self._publish_twist(speed, 0.0)
            time.sleep(period)

        self._stop()
        self._node.get_logger().info(f"Drive complete ({elapsed:.1f}s)")
        return True

    def _rotate_angle(self, angle_rad, speed):
        """Rotate for angle/speed seconds at constant angular speed.

        Uses time-based control because the wheel odometry may not be
        accurately calibrated.

        Args:
            angle_rad: Angle to rotate in radians (positive = CCW).
            speed: Rotation speed in rad/s.

        Returns:
            True when the timed rotation completes.
        """
        sign = 1.0 if angle_rad >= 0 else -1.0
        rotate_time = abs(angle_rad) / speed
        period = 1.0 / self._publish_rate
        start_time = time.time()

        self._node.get_logger().info(
            f"Rotate: {math.degrees(angle_rad):.0f} deg at "
            f"{math.degrees(speed):.0f} deg/s for {rotate_time:.1f}s"
        )

        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed >= rotate_time:
                break
            self._publish_twist(0.0, sign * speed)
            time.sleep(period)

        self._stop()
        self._node.get_logger().info(f"Rotate complete ({elapsed:.1f}s)")
        return True

    def _pause(self, duration_sec):
        """Publish zero velocity for a specified duration.

        Args:
            duration_sec: Duration to pause in seconds.
        """
        period = 1.0 / self._publish_rate
        iterations = int(duration_sec * self._publish_rate)

        for _ in range(iterations):
            if not rclpy.ok():
                break
            self._publish_twist(0.0, 0.0)
            time.sleep(period)

    def _publish_twist(self, linear_x, angular_z):
        """Publish a TwistStamped message.

        Args:
            linear_x: Linear velocity in m/s.
            angular_z: Angular velocity in rad/s.
        """
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def _stop(self):
        """Publish a zero-velocity command."""
        self._publish_twist(0.0, 0.0)

    def _normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi].

        Args:
            angle: Angle in radians.

        Returns:
            Normalized angle in radians.
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ── Sensor logging ────────────────────────────────────────────────

    def start_logging(self):
        """Start recording odometry and IMU data."""
        self._odom_log = []
        self._imu_log = []
        self._logging_active = True

        # Subscribe to IMU if not already subscribed
        if self._imu_sub is None:
            try:
                self._imu_sub = self._node.create_subscription(
                    Imu,
                    "/imu/data",
                    self._imu_callback,
                    10,
                )
            except Exception as e:
                self._node.get_logger().warn(f"Could not subscribe to /imu/data: {e}")

    def stop_logging(self):
        """Stop recording and return logged data as JSON strings.

        Returns:
            Tuple of (odom_log_json, imu_log_json) where each is a JSON
            string. odom_log entries have keys: timestamp, x, y, yaw, vx,
            vyaw. imu_log entries have keys: timestamp, ax, ay, az, gx,
            gy, gz.
        """
        self._logging_active = False

        odom_json = json.dumps(self._odom_log)
        imu_json = json.dumps(self._imu_log)

        return odom_json, imu_json
