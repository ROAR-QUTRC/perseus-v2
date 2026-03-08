"""Odometry verification tool for Perseus Lite.

Drives the robot through known motions and asks the operator to measure
the actual result, then reports odometry accuracy.  Physical parameters
(wheel_radius, wheel_separation) are NOT modified — they are fixed
quantities that should be set from tape-measure readings in
perseus_lite_controllers.yaml.

Usage:
    ROS_DOMAIN_ID=42 ros2 run mapping_autotune calibrate_odom
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.srv import GetParameters


CONTROLLER_NODE = "/diff_drive_base_controller"

LINEAR_SPEED = 0.2  # m/s — gentle speed for indoor testing
ROTATION_SPEED = 0.5  # rad/s
PUBLISH_RATE = 20.0  # Hz
COMMANDED_DISTANCE = 1.0  # metres per linear trial
COMMANDED_ANGLE = math.pi / 2.0  # 90 degrees per rotation trial
ACCEPTABLE_LINEAR_ERROR = 0.05  # metres
ACCEPTABLE_ROTATION_ERROR = 5.0  # degrees


class VerificationNode(Node):
    def __init__(self):
        super().__init__("odom_verification")
        self.declare_parameter("servo_max_rpm", 112.8)
        self._servo_max_rpm = self.get_parameter("servo_max_rpm").value

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


def _prompt_float(prompt_text):
    while True:
        try:
            return float(input(prompt_text))
        except ValueError:
            print("  Invalid number, try again.")


def _prompt_choice(prompt_text, valid):
    while True:
        choice = input(prompt_text).strip()
        if choice in valid:
            return choice
        print(f"  Please enter one of: {', '.join(valid)}")


# ── Linear verification ──────────────────────────────────────────

def run_linear_verification(node):
    """Drive a known distance and compare to operator measurement."""
    print("\n" + "=" * 60)
    print("CHECK A — Linear distance accuracy")
    print("=" * 60)
    print(f"  The robot will drive what it thinks is {COMMANDED_DISTANCE:.1f} m.")
    print("  Measure the actual distance with a tape measure.\n")

    drive_time = COMMANDED_DISTANCE / LINEAR_SPEED
    print(f"  Commanding {COMMANDED_DISTANCE:.1f} m forward "
          f"({LINEAR_SPEED} m/s for {drive_time:.1f}s)")

    input("  Press ENTER when ready to drive...")

    node.drive_timed(LINEAR_SPEED, drive_time)
    time.sleep(0.5)

    measured = _prompt_float(
        "  How far did the robot actually travel? (metres, e.g. 0.95): "
    )

    if measured <= 0:
        print("  Distance must be positive.")
        return None

    error = measured - COMMANDED_DISTANCE
    error_pct = (error / COMMANDED_DISTANCE) * 100.0

    print(f"\n  Commanded: {COMMANDED_DISTANCE:.3f} m")
    print(f"  Measured:  {measured:.3f} m")
    print(f"  Error:     {error:+.3f} m ({error_pct:+.1f}%)")

    if abs(error) <= ACCEPTABLE_LINEAR_ERROR:
        print("  PASS — linear odometry is within tolerance.")
    else:
        # Compute corrected servo_max_rpm based on actual vs commanded distance
        corrected_rpm = node._servo_max_rpm * (measured / COMMANDED_DISTANCE)
        print(f"\n  FAIL — linear error exceeds tolerance.")
        print(f"  Current servo_max_rpm: {node._servo_max_rpm:.1f}")
        print(f"  Suggested servo_max_rpm: {corrected_rpm:.1f}")
        print(f"\n  To fix, set servo_max_rpm:={corrected_rpm:.1f} in your launch command,")
        print(f"  or pass it as a hardware parameter in the URDF.")

    return {"commanded": COMMANDED_DISTANCE, "measured": measured,
            "error_m": error, "error_pct": error_pct}


# ── Rotation verification ────────────────────────────────────────

def run_rotation_verification(node):
    """Command a 90-degree rotation and ask operator to judge the result."""
    print("\n" + "=" * 60)
    print("CHECK B — Rotation accuracy")
    print("=" * 60)
    print("  The robot will attempt a 90-degree rotation.")
    print("  Judge whether it turned too little, about right, or too much.\n")

    rotate_time = COMMANDED_ANGLE / ROTATION_SPEED
    print(f"  Commanding 90 deg rotation "
          f"({math.degrees(ROTATION_SPEED):.0f} deg/s for {rotate_time:.1f}s)")

    input("  Press ENTER when ready to rotate...")

    node.rotate_timed(ROTATION_SPEED, rotate_time)
    time.sleep(0.5)

    choice = _prompt_choice(
        "  Was the rotation:\n"
        "    [1] Too little  (under-rotated)\n"
        "    [2] Just right  (close to 90 deg)\n"
        "    [3] Too much    (over-rotated)\n"
        "  Enter 1/2/3: ",
        {"1", "2", "3"},
    )

    label = {"1": "under-rotated", "2": "correct", "3": "over-rotated"}[choice]
    print(f"\n  Result: {label}")

    if choice == "2":
        print("  PASS — rotation odometry looks correct.")
    elif choice == "1":
        print("  FAIL — under-rotation.  wheel_separation in the controller")
        print("         config is likely too small.  Measure the centre-to-centre")
        print("         distance between left and right wheels and update")
        print("         perseus_lite_controllers.yaml.")
    else:
        print("  FAIL — over-rotation.  wheel_separation in the controller")
        print("         config is likely too large.  Measure the centre-to-centre")
        print("         distance between left and right wheels and update")
        print("         perseus_lite_controllers.yaml.")

    return {"commanded_deg": 90.0, "result": label}


# ── Main ────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = VerificationNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print()
    print("=" * 60)
    print("  Perseus Lite — Odometry Verification")
    print("=" * 60)
    print()
    print("This tool verifies that the configured wheel_radius and")
    print("wheel_separation match the physical robot.  It does NOT")
    print("modify controller parameters — those should be set from")
    print("direct tape-measure readings in perseus_lite_controllers.yaml.")
    print()
    print("Requirements:")
    print("  - Perseus Lite stack is running (controllers loaded)")
    print("  - Robot is on the ground with ~2 m of clear space ahead")
    print("  - You have a tape measure for linear distance")
    print()

    # Show current controller values
    print("Reading current values from diff_drive_base_controller...")
    ctrl_radius, ctrl_separation = node.get_controller_params()
    if ctrl_radius is not None and ctrl_separation is not None:
        print(f"  wheel_radius:     {ctrl_radius:.4f} m")
        print(f"  wheel_separation: {ctrl_separation:.4f} m")
    else:
        print("  Could not read from controller.")
    print(f"  servo_max_rpm:    {node._servo_max_rpm:.1f}")

    print()

    try:
        linear_result = run_linear_verification(node)
        rotation_result = run_rotation_verification(node)

        print()
        print("=" * 60)
        print("  VERIFICATION SUMMARY")
        print("=" * 60)
        print()
        if linear_result:
            print(f"  Linear:   {linear_result['error_pct']:+.1f}% error "
                  f"({linear_result['error_m']:+.3f} m)")
        if rotation_result:
            print(f"  Rotation: {rotation_result['result']}")
        print()
        print("  If either check failed, measure the physical dimensions")
        print("  with a tape measure and update perseus_lite_controllers.yaml:")
        print("    wheel_radius:     half the wheel diameter")
        print("    wheel_separation: centre-to-centre between left and right wheels")
        print()

    except KeyboardInterrupt:
        print("\n\nVerification aborted by user.")
        node.stop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
