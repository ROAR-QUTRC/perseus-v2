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

        if pattern == "corridor":
            return self._execute_corridor(linear_speed, rotation_speed)
        else:
            return self._execute_box_return(linear_speed, rotation_speed)

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
        """Drive forward a specified distance using closed-loop P control.

        Args:
            distance: Distance to travel in meters (positive = forward).
            speed: Maximum forward speed in m/s.

        Returns:
            True if target distance reached within tolerance, False on timeout.
        """
        start_x = self._current_x
        start_y = self._current_y

        kp = 2.0
        timeout_sec = (distance / speed) * 3.0
        start_time = time.time()

        rate = self._node.create_rate(self._publish_rate)

        while rclpy.ok():
            # Check timeout
            elapsed = time.time() - start_time
            if elapsed > timeout_sec:
                self._node.get_logger().warn(
                    f"Drive distance timeout after {elapsed:.1f}s "
                    f"(target: {distance:.2f}m)"
                )
                self._stop()
                return False

            # Compute distance traveled
            dx = self._current_x - start_x
            dy = self._current_y - start_y
            traveled = math.sqrt(dx * dx + dy * dy)

            remaining = distance - traveled

            # Check if we reached the target
            if remaining <= self._position_tolerance:
                self._stop()
                return True

            # P-controller: command proportional to remaining distance
            cmd_speed = min(speed, kp * remaining)
            self._publish_twist(cmd_speed, 0.0)

            rate.sleep()

        self._stop()
        return False

    def _rotate_angle(self, angle_rad, speed):
        """Rotate by a specified angle using closed-loop P control.

        Args:
            angle_rad: Angle to rotate in radians (positive = CCW).
            speed: Maximum rotation speed in rad/s.

        Returns:
            True if target angle reached within tolerance, False on timeout.
        """
        start_yaw = self._current_yaw
        cumulative_angle = 0.0
        prev_yaw = start_yaw

        sign = 1.0 if angle_rad >= 0 else -1.0
        target_angle = abs(angle_rad)
        kp = 3.0
        min_cmd = 0.08  # Minimum command to overcome static friction

        timeout_sec = (abs(angle_rad) / speed) * 3.0
        start_time = time.time()

        rate = self._node.create_rate(self._publish_rate)

        while rclpy.ok():
            # Check timeout
            elapsed = time.time() - start_time
            if elapsed > timeout_sec:
                self._node.get_logger().warn(
                    f"Rotate angle timeout after {elapsed:.1f}s "
                    f"(target: {math.degrees(angle_rad):.1f} deg)"
                )
                self._stop()
                return False

            # Track cumulative angle change with wraparound handling
            delta_yaw = self._normalize_angle(self._current_yaw - prev_yaw)
            cumulative_angle += delta_yaw
            prev_yaw = self._current_yaw

            remaining = target_angle - abs(cumulative_angle)

            # Check if we reached the target
            if remaining <= self._angle_tolerance:
                self._stop()
                return True

            # P-controller with deadband compensation
            cmd_speed = min(speed, kp * abs(remaining))
            cmd_speed = max(cmd_speed, min_cmd)
            self._publish_twist(0.0, sign * cmd_speed)

            rate.sleep()

        self._stop()
        return False

    def _pause(self, duration_sec):
        """Publish zero velocity for a specified duration.

        Args:
            duration_sec: Duration to pause in seconds.
        """
        rate = self._node.create_rate(self._publish_rate)
        iterations = int(duration_sec * self._publish_rate)

        for _ in range(iterations):
            if not rclpy.ok():
                break
            self._publish_twist(0.0, 0.0)
            rate.sleep()

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
