"""Odometry calibration tool for Perseus Lite.

Drives the robot through known motions and asks the operator to measure
the actual result, then computes and optionally applies corrections.

Tunable parameters:
  - servo_max_rpm:    Scales protocol units <-> real velocity. Varies with
                      supply voltage and is not directly measurable.
                      Calibrated via the linear (straight-line) test.
  - wheel_separation: The *effective* track width used by the diff-drive
                      controller. The physical tape-measure value is a
                      starting point, but wheel scrub, tire deformation,
                      and contact-patch geometry mean the effective value
                      often differs by 5-10%. Calibrated via the rotation test.

Fixed parameters (not tunable):
  - wheel_radius:     Redundant with servo_max_rpm for linear scaling --
                      tuning servo_max_rpm absorbs any effective-radius error.
  - Encoder resolution (4096 ticks/rev) and protocol max (32767) are
    hardware/protocol constants.

Usage:
    # Run just the straight-line (linear) calibration:
    ROS_DOMAIN_ID=42 ros2 run mapping_autotune calibrate_odom \\
        --ros-args -p test:=linear

    # Run just the rotation calibration:
    ROS_DOMAIN_ID=42 ros2 run mapping_autotune calibrate_odom \\
        --ros-args -p test:=rotation

    # Run both (default -- linear first, then rotation):
    ROS_DOMAIN_ID=42 ros2 run mapping_autotune calibrate_odom

    # Specify workspace source directory explicitly (if auto-detect fails):
    ROS_DOMAIN_ID=42 ros2 run mapping_autotune calibrate_odom \\
        --ros-args -p src_dir:=/home/dingo/perseus-v2/software/ros_ws/src
"""

import math
import os
import re
import subprocess
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.srv import GetParameters


CONTROLLER_NODE = "/diff_drive_base_controller"

LINEAR_SPEED = 0.2  # m/s -- gentle speed for indoor testing
ROTATION_SPEED = 0.5  # rad/s
PUBLISH_RATE = 20.0  # Hz
COMMANDED_DISTANCE = 1.0  # metres per linear trial
COMMANDED_ANGLE = 360.0  # degrees per rotation trial (full rotation)
ACCEPTABLE_LINEAR_ERROR = 0.05  # metres
ACCEPTABLE_ROTATION_ERROR = 5.0  # degrees

# Relative paths within the ROS workspace src directory
_LAUNCH_REL_PATHS = [
    "perseus_lite/launch/perseus_lite.launch.py",
    "perseus_lite/launch/perseus_lite_slam_and_nav2.launch.py",
]
_XACRO_REL_PATH = "perseus_lite/urdf/perseus_lite.urdf.xacro"
_CONTROLLERS_REL_PATH = "perseus_lite/config/perseus_lite_controllers.yaml"


# ── Source directory detection ──────────────────────────────────


def _find_src_dir(explicit_path=None):
    """Find the ROS workspace src directory.

    Strategy:
      1. Use explicit_path if provided and valid.
      2. Try git rev-parse to find repo root, then derive src dir.
      3. Return None (caller prints manual instructions).
    """
    if explicit_path and os.path.isdir(explicit_path):
        test_file = os.path.join(explicit_path, _CONTROLLERS_REL_PATH)
        if os.path.isfile(test_file):
            return explicit_path

    # Try git root detection
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode == 0:
            repo_root = result.stdout.strip()
            src_dir = os.path.join(repo_root, "software", "ros_ws", "src")
            test_file = os.path.join(src_dir, _CONTROLLERS_REL_PATH)
            if os.path.isfile(test_file):
                return src_dir
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    return None


def _read_servo_max_rpm_from_source(src_dir):
    """Read the current servo_max_rpm default from the xacro file."""
    if src_dir is None:
        return None
    xacro_path = os.path.join(src_dir, _XACRO_REL_PATH)
    if not os.path.isfile(xacro_path):
        return None
    try:
        with open(xacro_path, "r") as f:
            content = f.read()
        match = re.search(
            r'<xacro:arg\s+name="servo_max_rpm"\s+default="([^"]*)"', content
        )
        if match:
            return float(match.group(1))
    except (ValueError, OSError):
        pass
    return None


def _read_wheel_separation_from_source(src_dir):
    """Read the current wheel_separation from controllers yaml."""
    if src_dir is None:
        return None
    yaml_path = os.path.join(src_dir, _CONTROLLERS_REL_PATH)
    if not os.path.isfile(yaml_path):
        return None
    try:
        with open(yaml_path, "r") as f:
            content = f.read()
        match = re.search(r"wheel_separation:\s*([0-9]+\.?[0-9]*)", content)
        if match:
            return float(match.group(1))
    except (ValueError, OSError):
        pass
    return None


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("odom_calibration")
        self.declare_parameter("servo_max_rpm", 0.0)  # 0 = auto-detect from source
        self.declare_parameter("test", "both")  # linear | rotation | both
        self.declare_parameter("src_dir", "")
        self._test_type = self.get_parameter("test").value

        explicit = self.get_parameter("src_dir").value
        self._src_dir = _find_src_dir(explicit if explicit else None)

        # Resolve servo_max_rpm: explicit param > source files > fallback
        param_rpm = self.get_parameter("servo_max_rpm").value
        if param_rpm > 0:
            self._servo_max_rpm = param_rpm
            self._rpm_source = "parameter"
        else:
            source_rpm = _read_servo_max_rpm_from_source(self._src_dir)
            if source_rpm is not None:
                self._servo_max_rpm = source_rpm
                self._rpm_source = "source files"
            else:
                self._servo_max_rpm = 112.8
                self._rpm_source = "default (could not read source)"

        self._cmd_pub = self.create_publisher(TwistStamped, "/joy_vel", 10)
        self._period = 1.0 / PUBLISH_RATE

        self._get_param_client = self.create_client(
            GetParameters, f"{CONTROLLER_NODE}/get_parameters"
        )

    # ── Parameter helpers ───────────────────────────────────────────

    def _call_service(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(f"Service {client.srv_name} not available")
            return None
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def get_controller_params(self):
        """Read wheel_radius and wheel_separation from the running controller."""
        request = GetParameters.Request()
        request.names = ["wheel_radius", "wheel_separation"]
        result = self._call_service(self._get_param_client, request)
        if result is None or len(result.values) < 2:
            return None, None
        return result.values[0].double_value, result.values[1].double_value

    # ── Motion commands ─────────────────────────────────────────────

    def publish_twist(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def stop(self):
        self.publish_twist(0.0, 0.0)

    def drive_timed(self, speed, duration):
        start = time.time()
        while rclpy.ok() and (time.time() - start) < duration:
            self.publish_twist(speed, 0.0)
            time.sleep(self._period)
        self.stop()

    def rotate_timed(self, angular_speed, duration):
        start = time.time()
        while rclpy.ok() and (time.time() - start) < duration:
            self.publish_twist(0.0, angular_speed)
            time.sleep(self._period)
        self.stop()


# ── Input helpers ───────────────────────────────────────────────


def _prompt_float(prompt_text):
    while True:
        try:
            return float(input(prompt_text))
        except ValueError:
            print("  Invalid number, try again.")


def _prompt_choice(prompt_text, valid):
    while True:
        choice = input(prompt_text).strip().lower()
        if choice in valid:
            return choice
        print(f"  Please enter one of: {', '.join(valid)}")


# ── File updaters ───────────────────────────────────────────────


def _update_servo_max_rpm_defaults(src_dir, new_rpm):
    """Update servo_max_rpm default_value in launch files and xacro."""
    if src_dir is None:
        return []

    updated_files = []

    for rel_path in _LAUNCH_REL_PATHS:
        filepath = os.path.join(src_dir, rel_path)
        if not os.path.isfile(filepath):
            continue
        with open(filepath, "r") as f:
            content = f.read()
        pattern = r'("servo_max_rpm",\s*\n\s*default_value=")[^"]*(")'
        new_content, count = re.subn(pattern, rf"\g<1>{new_rpm:.1f}\2", content)
        if count > 0:
            with open(filepath, "w") as f:
                f.write(new_content)
            updated_files.append(filepath)

    xacro_path = os.path.join(src_dir, _XACRO_REL_PATH)
    if os.path.isfile(xacro_path):
        with open(xacro_path, "r") as f:
            content = f.read()
        pattern = r'(<xacro:arg\s+name="servo_max_rpm"\s+default=")[^"]*(")'
        new_content, count = re.subn(pattern, rf"\g<1>{new_rpm:.1f}\2", content)
        if count > 0:
            with open(xacro_path, "w") as f:
                f.write(new_content)
            updated_files.append(xacro_path)

    return updated_files


def _update_wheel_separation(src_dir, new_separation):
    """Update wheel_separation in perseus_lite_controllers.yaml."""
    if src_dir is None:
        return []

    yaml_path = os.path.join(src_dir, _CONTROLLERS_REL_PATH)
    if not os.path.isfile(yaml_path):
        return []

    with open(yaml_path, "r") as f:
        content = f.read()

    pattern = r"(wheel_separation:\s*)[0-9]+\.?[0-9]*"
    new_content, count = re.subn(pattern, rf"\g<1>{new_separation:.4f}", content)
    if count > 0:
        with open(yaml_path, "w") as f:
            f.write(new_content)
        return [yaml_path]
    return []


# ── Linear calibration (servo_max_rpm) ──────────────────────────


def run_linear_calibration(node):
    """Drive 1 m, measure actual distance, compute corrected servo_max_rpm."""
    print("\n" + "=" * 60)
    print("  LINEAR CALIBRATION — servo_max_rpm")
    print("=" * 60)
    print(f"  The robot will drive what it thinks is {COMMANDED_DISTANCE:.1f} m.")
    print("  Measure the actual distance with a tape measure.")
    print(f"\n  Current servo_max_rpm: {node._servo_max_rpm:.1f} ({node._rpm_source})")

    drive_time = COMMANDED_DISTANCE / LINEAR_SPEED

    print(
        f"\n  Commanding {COMMANDED_DISTANCE:.1f} m forward "
        f"({LINEAR_SPEED} m/s for {drive_time:.1f} s)"
    )
    input("  Press ENTER when ready to drive...")

    node.drive_timed(LINEAR_SPEED, drive_time)
    time.sleep(0.5)

    measured = _prompt_float(
        "  How far did the robot actually travel? (metres, e.g. 0.95): "
    )
    while measured <= 0:
        print("  Distance must be positive.")
        measured = _prompt_float(
            "  How far did the robot actually travel? (metres, e.g. 0.95): "
        )

    error = measured - COMMANDED_DISTANCE
    error_pct = (error / COMMANDED_DISTANCE) * 100.0

    print(f"\n  Commanded: {COMMANDED_DISTANCE:.3f} m")
    print(f"  Measured:  {measured:.3f} m")
    print(f"  Error:     {error:+.3f} m ({error_pct:+.1f}%)")

    if abs(error) <= ACCEPTABLE_LINEAR_ERROR:
        print("  PASS — linear odometry is within tolerance.")
        return {
            "servo_max_rpm": node._servo_max_rpm,
            "commanded": COMMANDED_DISTANCE,
            "measured": measured,
            "error_m": error,
            "error_pct": error_pct,
            "updated_files": [],
        }

    corrected_rpm = node._servo_max_rpm * (measured / COMMANDED_DISTANCE)
    print(f"\n  FAIL — linear error exceeds {ACCEPTABLE_LINEAR_ERROR} m tolerance.")
    print(f"  Current servo_max_rpm:   {node._servo_max_rpm:.1f}")
    print(f"  Corrected servo_max_rpm: {corrected_rpm:.1f}")

    choice = _prompt_choice(
        "\n  [s] Save corrected value to source files\n"
        "  [q] Quit without saving\n"
        "  Enter s/q: ",
        {"s", "q"},
    )

    updated = []
    if choice == "s":
        updated = _update_servo_max_rpm_defaults(node._src_dir, corrected_rpm)
        if updated:
            print(f"\n  Updated servo_max_rpm to {corrected_rpm:.1f} in:")
            for f in updated:
                print(f"    {f}")
        else:
            print("  Warning: could not find source files to update.")
            if node._src_dir is None:
                print("  Pass -p src_dir:=/path/to/ros_ws/src to fix this.")
            print(
                f"\n  To apply manually, set servo_max_rpm default to {corrected_rpm:.1f} in:"
            )
            for p in _LAUNCH_REL_PATHS + [_XACRO_REL_PATH]:
                print(f"    {p}")
    else:
        print(f"\n  Not saved. Corrected value: servo_max_rpm:={corrected_rpm:.1f}")

    if updated:
        print("\n  Rebuild and restart the Perseus Lite stack, then re-run")
        print("  this calibration to verify the new value.")

    return {
        "servo_max_rpm": corrected_rpm,
        "commanded": COMMANDED_DISTANCE,
        "measured": measured,
        "error_m": error,
        "error_pct": error_pct,
        "updated_files": updated,
    }


# ── Rotation calibration (wheel_separation) ─────────────────────


def run_rotation_calibration(node, current_separation):
    """Command a full rotation, measure actual angle, compute corrected wheel_separation."""
    commanded_deg = COMMANDED_ANGLE
    commanded_rad = math.radians(commanded_deg)

    print("\n" + "=" * 60)
    print("  ROTATION CALIBRATION — effective wheel_separation")
    print("=" * 60)
    print(f"  The robot will attempt a {commanded_deg:.0f}-degree rotation.")
    print("  Measure the actual angle it turns.")
    print()
    print("  TIP: Place a piece of tape on the ground aligned with the")
    print("  front of the robot. After the rotation, measure the angle")
    print("  between the tape and the robot's new heading. For a full")
    print("  rotation, mark the start heading and see how far off it")
    print("  is when it comes back around.")
    print(f"\n  Current wheel_separation: {current_separation:.4f} m")

    rotate_time = commanded_rad / ROTATION_SPEED

    print(
        f"\n  Commanding {commanded_deg:.0f} deg rotation "
        f"({math.degrees(ROTATION_SPEED):.0f} deg/s for {rotate_time:.1f} s)"
    )
    input("  Press ENTER when ready to rotate...")

    node.rotate_timed(ROTATION_SPEED, rotate_time)
    time.sleep(0.5)

    measured_deg = _prompt_float(
        "  How many degrees did the robot actually rotate? "
        f"(e.g. {commanded_deg - 10:.0f}, {commanded_deg:.0f}, {commanded_deg + 10:.0f}): "
    )
    while measured_deg <= 0:
        print("  Angle must be positive.")
        measured_deg = _prompt_float(
            "  How many degrees did the robot actually rotate? "
            f"(e.g. {commanded_deg - 10:.0f}, {commanded_deg:.0f}, {commanded_deg + 10:.0f}): "
        )

    error_deg = measured_deg - commanded_deg
    error_pct = (error_deg / commanded_deg) * 100.0

    print(f"\n  Commanded: {commanded_deg:.1f} deg")
    print(f"  Measured:  {measured_deg:.1f} deg")
    print(f"  Error:     {error_deg:+.1f} deg ({error_pct:+.1f}%)")

    if abs(error_deg) <= ACCEPTABLE_ROTATION_ERROR:
        print("  PASS — rotation odometry is within tolerance.")
        return {
            "wheel_separation": current_separation,
            "commanded_deg": commanded_deg,
            "measured_deg": measured_deg,
            "error_deg": error_deg,
            "error_pct": error_pct,
            "updated_files": [],
        }

    # wheel_separation correction:
    # If the robot over-rotates, the effective track is narrower than
    # configured -> decrease wheel_separation.
    # If under-rotates, effective track is wider -> increase it.
    corrected_sep = current_separation * (commanded_deg / measured_deg)

    print(
        f"\n  FAIL — rotation error exceeds {ACCEPTABLE_ROTATION_ERROR:.0f} deg tolerance."
    )
    print(f"  Current wheel_separation:   {current_separation:.4f} m")
    print(f"  Corrected wheel_separation: {corrected_sep:.4f} m")
    if error_deg > 0:
        print("  (Robot over-rotated -> effective track width is narrower)")
    else:
        print("  (Robot under-rotated -> effective track width is wider)")

    choice = _prompt_choice(
        "\n  [s] Save corrected value to controllers yaml\n"
        "  [q] Quit without saving\n"
        "  Enter s/q: ",
        {"s", "q"},
    )

    updated = []
    if choice == "s":
        updated = _update_wheel_separation(node._src_dir, corrected_sep)
        if updated:
            print(f"\n  Updated wheel_separation to {corrected_sep:.4f} in:")
            for f in updated:
                print(f"    {f}")
        else:
            print("  Warning: could not find source files to update.")
            if node._src_dir is None:
                print("  Pass -p src_dir:=/path/to/ros_ws/src to fix this.")
            print(f"\n  To apply manually, set wheel_separation: {corrected_sep:.4f}")
            print(f"    in {_CONTROLLERS_REL_PATH}")
    else:
        print(f"\n  Not saved. Corrected value: wheel_separation: {corrected_sep:.4f}")

    if updated:
        print("\n  Rebuild and restart the Perseus Lite stack, then re-run")
        print("  this calibration to verify the new value.")

    return {
        "wheel_separation": corrected_sep,
        "commanded_deg": commanded_deg,
        "measured_deg": measured_deg,
        "error_deg": error_deg,
        "error_pct": error_pct,
        "updated_files": updated,
    }


# ── Main ────────────────────────────────────────────────────────


def main():
    rclpy.init()
    node = CalibrationNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    test_type = node._test_type
    if test_type not in ("linear", "rotation", "both"):
        print(f"  Unknown test type '{test_type}'. Use: linear, rotation, both")
        node.destroy_node()
        rclpy.try_shutdown()
        return

    print()
    print("=" * 60)
    print("  Perseus Lite — Odometry Calibration")
    print("=" * 60)
    print()
    print("Tunable parameters:")
    print("  servo_max_rpm      — calibrated via straight-line test")
    print("  wheel_separation   — calibrated via rotation test (effective value)")
    print()
    print("Fixed parameters (not tunable here):")
    print("  wheel_radius       — redundant with servo_max_rpm")
    print("  encoder resolution — hardware constant (4096 ticks/rev)")
    print()
    if test_type == "both":
        print("IMPORTANT: Run the linear test FIRST. servo_max_rpm affects")
        print("all wheel velocities equally, so it must be correct before")
        print("the rotation test can isolate wheel_separation error.")
        print()
    print("Requirements:")
    print("  - Perseus Lite stack is running (controllers loaded)")
    print("  - Robot is on the ground with ~2 m of clear space ahead")
    print("  - You have a tape measure and a way to measure angles")
    print()

    # Show source directory detection status
    if node._src_dir:
        print(f"Source directory: {node._src_dir}")
    else:
        print("Source directory: NOT FOUND (pass -p src_dir:=/path/to/ros_ws/src)")
        print("  Corrections will be printed but not auto-applied.")
    print()

    # Show current controller values
    print("Reading current values from diff_drive_base_controller...")
    ctrl_radius, ctrl_separation = node.get_controller_params()
    if ctrl_radius is not None and ctrl_separation is not None:
        print(f"  wheel_radius:     {ctrl_radius:.4f} m  (fixed)")
        print(f"  wheel_separation: {ctrl_separation:.4f} m  (tunable — effective)")
    else:
        # Fall back to reading from source files
        ctrl_separation = _read_wheel_separation_from_source(node._src_dir)
        if ctrl_separation is not None:
            print("  Could not read from controller. Using source file values.")
            print(f"  wheel_separation: {ctrl_separation:.4f} m  (from source)")
        else:
            print("  Could not read from controller or source files.")
    print(f"  servo_max_rpm:    {node._servo_max_rpm:.1f}  ({node._rpm_source})")
    print()

    linear_result = None
    rotation_result = None

    try:
        if test_type in ("linear", "both"):
            linear_result = run_linear_calibration(node)

        if test_type in ("rotation", "both"):
            if ctrl_separation is None:
                print("\n  Cannot run rotation test — could not read")
                print("  wheel_separation from the controller or source files.")
            else:
                rotation_result = run_rotation_calibration(node, ctrl_separation)

        # ── Summary ─────────────────────────────────────────────
        print()
        print("=" * 60)
        print("  CALIBRATION SUMMARY")
        print("=" * 60)
        print()

        any_updated = False
        if linear_result:
            status = (
                "PASS"
                if abs(linear_result["error_m"]) <= ACCEPTABLE_LINEAR_ERROR
                else "CORRECTED"
            )
            print(
                f"  Linear:   {status}  {linear_result['error_pct']:+.1f}% error "
                f"({linear_result['error_m']:+.3f} m)"
            )
            print(f"            servo_max_rpm = {linear_result['servo_max_rpm']:.1f}")
            if linear_result["updated_files"]:
                any_updated = True

        if rotation_result:
            status = (
                "PASS"
                if abs(rotation_result["error_deg"]) <= ACCEPTABLE_ROTATION_ERROR
                else "CORRECTED"
            )
            print(
                f"  Rotation: {status}  {rotation_result['error_pct']:+.1f}% error "
                f"({rotation_result['error_deg']:+.1f} deg)"
            )
            print(
                f"            wheel_separation = {rotation_result['wheel_separation']:.4f} m"
            )
            if rotation_result["updated_files"]:
                any_updated = True

        if any_updated:
            print()
            print("  Source files were updated. Rebuild and restart the")
            print("  Perseus Lite stack for changes to take effect.")
            print("  Then re-run this calibration to verify.")
        print()

    except KeyboardInterrupt:
        print("\n\nCalibration aborted by user.")
        node.stop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
