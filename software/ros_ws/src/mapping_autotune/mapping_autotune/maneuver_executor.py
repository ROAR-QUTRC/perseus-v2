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

        # Read servo_max_rpm from the node's parameters if available
        try:
            self._servo_max_rpm = node.get_parameter("servo_max_rpm").value
        except rclpy.exceptions.ParameterNotDeclaredException:
            self._servo_max_rpm = 62.0

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
        """Drive forward 1m, pause, then reverse 1m back to start.

        Sequence:
            1. Drive forward 1.0m
            2. Pause 1.0s
            3. Drive backward 1.0m

        Returns:
            True if all steps completed, False on any failure.
        """
        steps = [
            ("drive", 1.0, linear_speed),
            ("pause", 1.0, None),
            ("drive", -1.0, linear_speed),
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
        sign = 1.0 if distance >= 0 else -1.0
        drive_time = abs(distance) / speed
        period = 1.0 / self._publish_rate
        start_time = time.time()

        self._node.get_logger().info(
            f"Drive: speed={sign * speed:.2f} m/s for {drive_time:.1f}s (~{distance:.2f}m)"
        )

        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed >= drive_time:
                break
            self._publish_twist(sign * speed, 0.0)
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

    # ── TF2 setup for SLAM pose lookups ──────────────────────────────

    def setup_tf(self):
        """Initialize TF buffer and listener for SLAM pose lookups.

        Called lazily before Phase 0 calibration to avoid overhead when
        calibration is not needed.
        """
        from tf2_ros import Buffer, TransformListener

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

    def _get_slam_pose(self):
        """Get current SLAM pose via TF lookup (map -> base_link).

        Returns:
            Tuple of (x, y, yaw) or None on failure.
        """
        try:
            t = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
        except Exception as e:
            self._node.get_logger().warn(f"TF lookup failed: {e}")
            return None

    # ── Odometry calibration ─────────────────────────────────────────

    def calibration_drive(
        self,
        linear_speed,
        rotation_speed,
    ):
        """Drive a diagnostic pattern comparing odom vs SLAM ground truth.

        Drives straight for 5s then rotates 360 deg, recording odom and
        SLAM poses to measure odometry accuracy.  Reports error percentages
        but does not compute corrected wheel parameters — those are physical
        measurements that should be set from a tape measure.

        Args:
            linear_speed: Forward speed in m/s.
            rotation_speed: Rotation speed in rad/s.

        Returns:
            Dict with diagnostic results, or None on failure.
        """
        logger = self._node.get_logger()

        # Wait for initial odom and SLAM poses
        if not self._wait_for_odom(timeout=5.0):
            logger.error("Calibration: no odom data received")
            return None

        slam_start = self._get_slam_pose()
        if slam_start is None:
            logger.error("Calibration: could not get starting SLAM pose")
            return None

        odom_start_x = self._current_x
        odom_start_y = self._current_y

        logger.info(
            f"Calibration: start odom=({odom_start_x:.3f}, {odom_start_y:.3f}), "
            f"SLAM=({slam_start[0]:.3f}, {slam_start[1]:.3f})"
        )

        # ── Phase A: Drive straight for 5s ─────────────────────────
        drive_time = 5.0
        period = 1.0 / self._publish_rate
        start_time = time.time()

        logger.info(
            f"Calibration: driving straight at {linear_speed:.2f} m/s "
            f"for {drive_time:.1f}s"
        )

        while rclpy.ok() and (time.time() - start_time) < drive_time:
            self._publish_twist(linear_speed, 0.0)
            time.sleep(period)
        self._stop()

        # Record ending poses after linear drive
        time.sleep(0.5)  # Brief settle
        odom_end_x = self._current_x
        odom_end_y = self._current_y

        slam_end_linear = self._get_slam_pose()
        if slam_end_linear is None:
            logger.error("Calibration: could not get SLAM pose after linear drive")
            return None

        odom_linear_dist = math.sqrt(
            (odom_end_x - odom_start_x) ** 2 + (odom_end_y - odom_start_y) ** 2
        )
        slam_linear_dist = math.sqrt(
            (slam_end_linear[0] - slam_start[0]) ** 2
            + (slam_end_linear[1] - slam_start[1]) ** 2
        )

        logger.info(
            f"Calibration: linear drive done. "
            f"odom={odom_linear_dist:.3f}m, SLAM={slam_linear_dist:.3f}m"
        )

        # ── Pause 2s ───────────────────────────────────────────────
        self._pause(2.0)

        # ── Phase B: Rotate 360 deg ────────────────────────────────
        slam_rot_start = self._get_slam_pose()
        if slam_rot_start is None:
            logger.error("Calibration: could not get SLAM pose before rotation")
            return None

        rotate_time = (2.0 * math.pi) / rotation_speed
        start_time = time.time()

        logger.info(
            f"Calibration: rotating 360 deg at {math.degrees(rotation_speed):.0f} "
            f"deg/s for {rotate_time:.1f}s"
        )

        while rclpy.ok() and (time.time() - start_time) < rotate_time:
            self._publish_twist(0.0, rotation_speed)
            time.sleep(period)
        self._stop()

        # Record ending SLAM pose after rotation
        time.sleep(0.5)  # Brief settle
        slam_rot_end = self._get_slam_pose()
        if slam_rot_end is None:
            logger.error("Calibration: could not get SLAM pose after rotation")
            return None

        # Compute rotation angles.
        # For odom: use commanded time * speed since odom snapshots wrap at +-pi.
        odom_rotation_angle = rotation_speed * rotate_time

        # For SLAM: the normalized delta wraps near 0 for a full 360.
        # Pick the interpretation closest to the commanded 2pi.
        slam_raw_delta = self._normalize_angle(slam_rot_end[2] - slam_rot_start[2])
        slam_abs_delta = abs(slam_raw_delta)
        if slam_abs_delta < math.pi:
            slam_rotation_angle = 2.0 * math.pi - slam_abs_delta
        else:
            slam_rotation_angle = slam_abs_delta

        logger.info(
            f"Calibration: rotation done. "
            f"odom_angle={math.degrees(odom_rotation_angle):.1f} deg, "
            f"SLAM_angle={math.degrees(slam_rotation_angle):.1f} deg"
        )

        # ── Compute accuracy metrics ──────────────────────────────
        if odom_linear_dist < 0.01:
            logger.error("Calibration: odom linear distance too small")
            return None

        if slam_rotation_angle < 0.1:
            logger.error("Calibration: SLAM rotation angle too small")
            return None

        linear_error = slam_linear_dist - odom_linear_dist
        linear_error_pct = (linear_error / slam_linear_dist) * 100.0
        rotation_error = slam_rotation_angle - odom_rotation_angle
        rotation_error_pct = (rotation_error / slam_rotation_angle) * 100.0

        results = {
            "odom_linear_distance": odom_linear_dist,
            "slam_linear_distance": slam_linear_dist,
            "linear_error_m": linear_error,
            "linear_error_pct": linear_error_pct,
            "odom_rotation_angle": odom_rotation_angle,
            "slam_rotation_angle": slam_rotation_angle,
            "rotation_error_deg": math.degrees(rotation_error),
            "rotation_error_pct": rotation_error_pct,
        }

        logger.info("=" * 60)
        logger.info("ODOMETRY VERIFICATION RESULTS")
        logger.info("=" * 60)
        logger.info(
            f"  Linear:   odom={odom_linear_dist:.3f}m  "
            f"SLAM={slam_linear_dist:.3f}m  "
            f"error={linear_error:+.3f}m ({linear_error_pct:+.1f}%)"
        )
        logger.info(
            f"  Rotation: odom={math.degrees(odom_rotation_angle):.1f} deg  "
            f"SLAM={math.degrees(slam_rotation_angle):.1f} deg  "
            f"error={math.degrees(rotation_error):+.1f} deg ({rotation_error_pct:+.1f}%)"
        )
        if abs(linear_error_pct) > 5.0:
            # Compute suggested servo_max_rpm from SLAM/odom ratio
            if odom_linear_dist > 0.01:
                suggested_rpm = self._servo_max_rpm * (slam_linear_dist / odom_linear_dist)
                results["suggested_servo_max_rpm"] = suggested_rpm
                logger.warning(
                    f"  Linear error exceeds 5%. Current servo_max_rpm={self._servo_max_rpm:.1f}"
                )
                logger.warning(
                    f"  Suggested servo_max_rpm={suggested_rpm:.1f} "
                    f"(based on SLAM/odom ratio {slam_linear_dist / odom_linear_dist:.3f})"
                )
                logger.warning(
                    "  Set servo_max_rpm:=%.1f in your launch command to correct velocity scaling.",
                    suggested_rpm,
                )

        if abs(rotation_error_pct) > 5.0:
            logger.warning(
                "  Rotation error exceeds 5%.  Verify that wheel_separation in "
                "perseus_lite_controllers.yaml matches tape-measure readings."
            )

        if abs(linear_error_pct) <= 5.0 and abs(rotation_error_pct) <= 5.0:
            logger.info("  Odometry accuracy is within 5% — OK.")
        logger.info("=" * 60)

        return results

    def _wait_for_odom(self, timeout=5.0):
        """Wait until at least one odom message has been received.

        Args:
            timeout: Maximum seconds to wait.

        Returns:
            True if odom data is available, False on timeout.
        """
        start = time.time()
        while not self._odom_received and rclpy.ok():
            if time.time() - start > timeout:
                return False
            time.sleep(0.1)
        return self._odom_received

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
