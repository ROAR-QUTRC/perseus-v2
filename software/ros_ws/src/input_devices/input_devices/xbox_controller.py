#!/usr/bin/env python3

import rclpy
from typing import List
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rcl_interfaces.msg import (
    ParameterDescriptor,
    FloatingPointRange,
    SetParametersResult,
)
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy


class XboxController(Node):
    """
    A ROS2 node that processes Xbox controller inputs and publishes velocity commands.

    This node subscribes to joystick inputs and converts them into velocity commands
    for robot control. It includes two dead man's switches for different speed modes
    and configurable scaling factors through ROS2 parameters.

    Publishers:
        input_devices/cmd_vel (geometry_msgs/Twist or TwistStamped): Robot velocity commands

    Subscribers:
        joy (sensor_msgs/Joy): Raw joystick inputs

    Parameters:
        translation_scale (double): Base scaling for linear motion [m/s] (default: 0.25)
        rotation_scale (double): Base scaling for angular motion [rad/s] (default: 0.50)
        high_speed_multiplier (double): Multiplier for high-speed mode (default: 2.0)
        deadman_threshold (double): Threshold for dead man's switch activation (default: -0.95)
        use_stamped_msg (bool): Whether to publish TwistStamped instead of Twist (default: False)
    """

    def __init__(self):
        """Initialize the Xbox controller node with configurable parameters."""
        super().__init__("xbox_controller")

        # Xbox controller axis mappings
        self.LEFT_STICK_Y_AXIS = 1
        self.RIGHT_STICK_X_AXIS = 2
        self.RIGHT_TRIGGER_AXIS = 5  # Regular speed deadman switch
        self.LEFT_TRIGGER_AXIS = 6  # High speed deadman switch

        # Create range constraints as instance variables for use in callback
        self._speed_range = FloatingPointRange(from_value=0.0, to_value=10.0, step=0.01)
        self._multiplier_range = FloatingPointRange(
            from_value=1.0, to_value=5.0, step=0.1
        )
        self._threshold_range = FloatingPointRange(
            from_value=-1.0, to_value=0.0, step=0.01
        )

        # Declare and load parameters with proper constraints and descriptions
        self._declare_parameters()
        self._load_parameters()

        # Create appropriate publisher based on message type parameter
        if self.use_stamped_msg:
            self.velocity_publisher = self.create_publisher(
                TwistStamped, "input_devices/cmd_vel", 10
            )
        else:
            self.velocity_publisher = self.create_publisher(
                Twist, "input_devices/cmd_vel", 10
            )

        self.joy_subscriber = self.create_subscription(
            Joy, "joy", self.process_joystick_input, 10
        )

        self.get_logger().info(
            f"Xbox controller initialized with translation scale: {self.translation_scale}, "
            f"rotation scale: {self.rotation_scale}, "
            f"high speed multiplier: {self.high_speed_multiplier}, "
            f"using {'stamped' if self.use_stamped_msg else 'unstamped'} messages"
        )

    def _declare_parameters(self) -> None:
        """Declare all node parameters with proper constraints and descriptions."""
        # Parameter declarations
        self.declare_parameter(
            "translation_scale",
            0.25,
            ParameterDescriptor(
                description="Base scaling factor for linear motion in m/s",
                floating_point_range=[self._speed_range],
                read_only=False,
            ),
        )

        self.declare_parameter(
            "rotation_scale",
            0.50,
            ParameterDescriptor(
                description="Base scaling factor for angular motion in rad/s",
                floating_point_range=[self._speed_range],
                read_only=False,
            ),
        )

        self.declare_parameter(
            "high_speed_multiplier",
            2.0,
            ParameterDescriptor(
                description="Multiplier for high-speed mode",
                floating_point_range=[self._multiplier_range],
                read_only=False,
            ),
        )

        self.declare_parameter(
            "deadman_threshold",
            -0.95,
            ParameterDescriptor(
                description="Threshold for dead man's switch activation",
                floating_point_range=[self._threshold_range],
                read_only=False,
            ),
        )

        self.declare_parameter(
            "use_stamped_msg",
            False,
            ParameterDescriptor(
                description="Whether to publish TwistStamped instead of Twist messages",
                read_only=True,
            ),
        )
        self.add_on_set_parameters_callback(self._parameter_callback)

    def _parameter_callback(self, params: List[Parameter]) -> SetParametersResult:
        """Handle parameter updates during runtime.

        Args:
            params: List of parameters that are being updated

        Returns:
            SetParametersResult indicating success/failure of parameter updates
        """
        for param in params:
            if param.name == "high_speed_multiplier":
                if (
                    param.value < self._multiplier_range.from_value
                    or param.value > self._multiplier_range.to_value
                ):
                    return SetParametersResult(
                        successful=False, reason="Multiplier out of valid range"
                    )
                self.high_speed_multiplier = param.value

            elif param.name == "deadman_threshold":
                if (
                    param.value < self._threshold_range.from_value
                    or param.value > self._threshold_range.to_value
                ):
                    return SetParametersResult(
                        successful=False, reason="Threshold out of valid range"
                    )
                self.deadman_threshold = param.value

            elif param.name == "translation_scale":
                if (
                    param.value < self._speed_range.from_value
                    or param.value > self._speed_range.to_value
                ):
                    return SetParametersResult(
                        successful=False, reason="Translation scale out of valid range"
                    )
                self.translation_scale = param.value

            elif param.name == "rotation_scale":
                if (
                    param.value < self._speed_range.from_value
                    or param.value > self._speed_range.to_value
                ):
                    return SetParametersResult(
                        successful=False, reason="Rotation scale out of valid range"
                    )
                self.rotation_scale = param.value

        return SetParametersResult(successful=True)

    def _load_parameters(self) -> None:
        """Load all declared parameters into instance variables."""
        self.translation_scale = self.get_parameter("translation_scale").value
        self.rotation_scale = self.get_parameter("rotation_scale").value
        self.high_speed_multiplier = self.get_parameter("high_speed_multiplier").value
        self.deadman_threshold = self.get_parameter("deadman_threshold").value
        self.use_stamped_msg = self.get_parameter("use_stamped_msg").value

    def process_joystick_input(self, joy_msg: Joy) -> None:
        """
        Process incoming joystick messages and publish velocity commands.

        The node supports two speed modes:
        - Regular speed (RIGHT_TRIGGER): Normal operation
        - High speed (LEFT_TRIGGER): Operates at high_speed_multiplier times normal speed

        Args:
            joy_msg (sensor_msgs/Joy): The incoming joystick message
        """
        twist = Twist()

        # Check deadman switches
        is_regular_speed = (
            joy_msg.axes[self.RIGHT_TRIGGER_AXIS] <= self.deadman_threshold
        )
        is_high_speed = joy_msg.axes[self.LEFT_TRIGGER_AXIS] <= self.deadman_threshold

        if is_regular_speed or is_high_speed:
            # Calculate base movement
            linear_input = joy_msg.axes[self.LEFT_STICK_Y_AXIS]
            rotation_input = joy_msg.axes[self.RIGHT_STICK_X_AXIS]

            # Apply appropriate scaling based on which deadman switch is engaged
            speed_multiplier = self.high_speed_multiplier if is_high_speed else 1.0

            twist.linear.x = linear_input * self.translation_scale * speed_multiplier

            rotation_value = rotation_input * self.rotation_scale * speed_multiplier

            # Invert rotation direction when moving backwards
            twist.angular.z = (
                -1.0 * rotation_value if twist.linear.x < 0 else rotation_value
            )

        # Create and publish appropriate message type
        if self.use_stamped_msg:
            stamped_twist = TwistStamped()
            stamped_twist.header.stamp = self.get_clock().now().to_msg()
            stamped_twist.header.frame_id = "base_link"
            stamped_twist.twist = twist
            self.velocity_publisher.publish(stamped_twist)
        else:
            self.velocity_publisher.publish(twist)

        self._log_debug_info(joy_msg, twist, is_regular_speed, is_high_speed)

    def _log_debug_info(
        self, joy_msg: Joy, twist: Twist, is_regular_speed: bool, is_high_speed: bool
    ) -> None:
        """
        Log debug information about controller state and outputs.

        Args:
            joy_msg (sensor_msgs/Joy): The current joystick message
            twist (geometry_msgs/Twist): The calculated velocity command
            is_regular_speed (bool): State of regular speed deadman switch
            is_high_speed (bool): State of high speed deadman switch
        """
        self.get_logger().debug("Xbox Controller Axes:")
        for idx, axis_value in enumerate(joy_msg.axes):
            self.get_logger().debug(f"  Axis {idx}: {axis_value:.2f}")

        self.get_logger().debug("Velocity Output:")
        self.get_logger().debug(f"  Linear X: {twist.linear.x:.2f} m/s")
        self.get_logger().debug(f"  Angular Z: {twist.angular.z:.2f} rad/s")

        speed_mode = (
            "High Speed"
            if is_high_speed
            else "Regular Speed" if is_regular_speed else "Disabled"
        )
        self.get_logger().debug(f"  Speed Mode: {speed_mode}")

        self.get_logger().debug("------------------------")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    controller = XboxController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
