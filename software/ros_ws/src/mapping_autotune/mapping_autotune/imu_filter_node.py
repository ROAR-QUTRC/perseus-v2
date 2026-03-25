"""Lightweight ROS2 node that relays IMU data with bias subtraction and deadband filtering."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFilterNode(Node):
    """Relay IMU data with gyro bias subtraction and deadband filtering.

    Subscribes to /imu/data, applies bias correction and deadband filtering
    to angular velocity, and publishes the result to /imu/data_filtered.
    Optionally runs an auto-calibration phase on startup to determine bias
    offsets and a recommended deadband from collected samples.
    """

    def __init__(self):
        super().__init__("imu_filter_node")

        # Declare parameters
        self.declare_parameter("gyro_bias_x", 0.0)
        self.declare_parameter("gyro_bias_y", 0.0)
        self.declare_parameter("gyro_bias_z", 0.0)
        self.declare_parameter("gyro_deadband", 0.01)
        self.declare_parameter("calibration_duration", 30.0)
        self.declare_parameter("auto_calibrate", False)

        # Internal state
        self._calibration_results = None
        self._is_calibrating = False
        self._calibration_samples_x = []
        self._calibration_samples_y = []
        self._calibration_samples_z = []
        self._calibration_start_time = None

        # Publisher
        self._pub = self.create_publisher(Imu, "/imu/data_filtered", 10)

        # Check if auto-calibration is requested
        auto_calibrate = self.get_parameter("auto_calibrate").value

        if auto_calibrate:
            self._is_calibrating = True
            self.get_logger().info(
                "IMU auto-calibration enabled. Collecting data for "
                f"{self.get_parameter('calibration_duration').value:.1f}s..."
            )

        # Subscriber (always active; behavior differs during calibration)
        self._sub = self.create_subscription(Imu, "/imu/data", self._imu_callback, 10)

    def _imu_callback(self, msg: Imu):
        """Handle incoming IMU messages for calibration or normal relay."""
        if self._is_calibrating:
            self._handle_calibration_sample(msg)
            return

        self._relay_filtered(msg)

    def _handle_calibration_sample(self, msg: Imu):
        """Accumulate gyro samples during the calibration window."""
        now = self.get_clock().now()

        if self._calibration_start_time is None:
            self._calibration_start_time = now

        elapsed = (now - self._calibration_start_time).nanoseconds * 1e-9
        calibration_duration = self.get_parameter("calibration_duration").value

        # Collect the sample
        self._calibration_samples_x.append(msg.angular_velocity.x)
        self._calibration_samples_y.append(msg.angular_velocity.y)
        self._calibration_samples_z.append(msg.angular_velocity.z)

        if elapsed >= calibration_duration:
            self._finish_calibration()

    def _finish_calibration(self):
        """Compute bias and deadband from collected samples and update parameters."""
        samples_x = np.array(self._calibration_samples_x)
        samples_y = np.array(self._calibration_samples_y)
        samples_z = np.array(self._calibration_samples_z)

        bias_x = float(np.mean(samples_x))
        bias_y = float(np.mean(samples_y))
        bias_z = float(np.mean(samples_z))

        stddev_x = float(np.std(samples_x))
        stddev_y = float(np.std(samples_y))
        stddev_z = float(np.std(samples_z))

        recommended_deadband = 3.0 * max(stddev_x, stddev_y, stddev_z)
        num_samples = len(self._calibration_samples_x)

        self.get_logger().info(
            f"IMU Calibration: bias=[{bias_x:.6f}, {bias_y:.6f}, {bias_z:.6f}], "
            f"recommended_deadband={recommended_deadband:.6f}"
        )

        # Update parameters with calibrated values
        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "gyro_bias_x", rclpy.Parameter.Type.DOUBLE, bias_x
                ),
                rclpy.parameter.Parameter(
                    "gyro_bias_y", rclpy.Parameter.Type.DOUBLE, bias_y
                ),
                rclpy.parameter.Parameter(
                    "gyro_bias_z", rclpy.Parameter.Type.DOUBLE, bias_z
                ),
                rclpy.parameter.Parameter(
                    "gyro_deadband", rclpy.Parameter.Type.DOUBLE, recommended_deadband
                ),
            ]
        )

        # Store calibration results
        self._calibration_results = {
            "bias_x": bias_x,
            "bias_y": bias_y,
            "bias_z": bias_z,
            "deadband": recommended_deadband,
            "samples": num_samples,
        }

        # Clean up calibration state
        self._calibration_samples_x.clear()
        self._calibration_samples_y.clear()
        self._calibration_samples_z.clear()
        self._calibration_start_time = None
        self._is_calibrating = False

        self.get_logger().info(
            f"Calibration complete ({num_samples} samples). Now relaying filtered IMU data."
        )

    def _relay_filtered(self, msg: Imu):
        """Apply bias subtraction and deadband filtering, then publish."""
        bias_x = self.get_parameter("gyro_bias_x").value
        bias_y = self.get_parameter("gyro_bias_y").value
        bias_z = self.get_parameter("gyro_bias_z").value
        deadband = self.get_parameter("gyro_deadband").value

        # Subtract bias from angular velocity
        corrected_x = msg.angular_velocity.x - bias_x
        corrected_y = msg.angular_velocity.y - bias_y
        corrected_z = msg.angular_velocity.z - bias_z

        # Apply deadband: if magnitude is below threshold, zero all components
        magnitude = math.sqrt(corrected_x**2 + corrected_y**2 + corrected_z**2)

        if magnitude < deadband:
            corrected_x = 0.0
            corrected_y = 0.0
            corrected_z = 0.0

        # Build output message (copy everything, override angular velocity)
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity.x = corrected_x
        out.angular_velocity.y = corrected_y
        out.angular_velocity.z = corrected_z
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self._pub.publish(out)

    def get_calibration_results(self):
        """Return calibration results dict, or None if not yet calibrated.

        Returns:
            dict or None: If calibrated, a dict with keys:
                bias_x, bias_y, bias_z, deadband, samples.
        """
        return self._calibration_results

    def set_bias(self, x: float, y: float, z: float):
        """Update gyro bias parameters dynamically."""
        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "gyro_bias_x", rclpy.Parameter.Type.DOUBLE, x
                ),
                rclpy.parameter.Parameter(
                    "gyro_bias_y", rclpy.Parameter.Type.DOUBLE, y
                ),
                rclpy.parameter.Parameter(
                    "gyro_bias_z", rclpy.Parameter.Type.DOUBLE, z
                ),
            ]
        )

    def set_deadband(self, value: float):
        """Update gyro deadband parameter dynamically."""
        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "gyro_deadband", rclpy.Parameter.Type.DOUBLE, value
                ),
            ]
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
