#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class XboxController(Node):
    """
    A ROS2 node that processes Xbox controller inputs and publishes velocity commands.

    This node subscribes to joystick inputs and converts them into velocity commands
    for robot control. It includes safety features like a dead man's switch and
    configurable scaling factors.

    Publishers:
        input_devices/cmd_vel (geometry_msgs/Twist): Robot velocity commands

    Subscribers:
        joy (sensor_msgs/Joy): Raw joystick inputs

    Parameters:
        TRANSLATION_SCALE (float): Scaling factor for linear motion (default: 0.25)
        ROTATION_SCALE (float): Scaling factor for angular motion (default: 0.50)
        DEADMAN_THRESHOLD (float): Threshold for dead man's switch activation (default: -0.95)
    """

    def __init__(self):
        """Initialize the Xbox controller node."""
        super().__init__("xbox_controller")

        # Constants
        self.TRANSLATION_SCALE = 0.25
        self.ROTATION_SCALE = 0.50
        self.DEADMAN_THRESHOLD = -0.95

        # Xbox controller axis mappings
        self.LEFT_STICK_Y_AXIS = 1
        self.RIGHT_STICK_X_AXIS = 2
        self.RIGHT_TRIGGER_AXIS = 5

        # Publishers and subscribers
        self.velocity_publisher = self.create_publisher(
            Twist, "input_devices/cmd_vel", 10
        )
        self.joy_subscriber = self.create_subscription(
            Joy, "joy", self.process_joystick_input, 10
        )

    def process_joystick_input(self, joy_msg: Joy) -> None:
        """
        Process incoming joystick messages and publish velocity commands.

        Args:
            joy_msg (sensor_msgs/Joy): The incoming joystick message
        """
        twist = Twist()

        # Check if dead man's switch (right trigger) is engaged
        is_deadman_engaged = (
            joy_msg.axes[self.RIGHT_TRIGGER_AXIS] <= self.DEADMAN_THRESHOLD
        )

        if is_deadman_engaged:
            # Process movement only when dead man's switch is engaged
            twist.linear.x = (
                joy_msg.axes[self.LEFT_STICK_Y_AXIS] * self.TRANSLATION_SCALE
            )

            rotation_input = joy_msg.axes[self.RIGHT_STICK_X_AXIS] * self.ROTATION_SCALE

            # Invert rotation direction when moving backwards
            twist.angular.z = (
                -1.0 * rotation_input if twist.linear.x < 0 else rotation_input
            )

        self._log_debug_info(joy_msg, twist, is_deadman_engaged)
        self.velocity_publisher.publish(twist)

    def _log_debug_info(
        self, joy_msg: Joy, twist: Twist, is_deadman_engaged: bool
    ) -> None:
        """
        Log debug information about controller state and outputs.

        Args:
            joy_msg (sensor_msgs/Joy): The current joystick message
            twist (geometry_msgs/Twist): The calculated velocity command
            is_deadman_engaged (bool): Current state of dead man's switch
        """
        self.get_logger().debug("Xbox Controller Axes:")
        for idx, axis_value in enumerate(joy_msg.axes):
            self.get_logger().debug(f"  Axis {idx}: {axis_value:.2f}")

        self.get_logger().debug("Velocity Output:")
        self.get_logger().debug(f"  Linear X: {twist.linear.x:.2f} m/s")
        self.get_logger().debug(f"  Angular Z: {twist.angular.z:.2f} rad/s")
        self.get_logger().debug(
            f'  Dead Man\'s Switch: {"Engaged" if is_deadman_engaged else "Disengaged"}'
        )
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
