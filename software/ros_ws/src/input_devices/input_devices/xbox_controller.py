import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class XboxControllerNode(Node):
    def __init__(self):
        super().__init__("xbox_controller_node")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscription = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.subscription  # prevent unused variable warning

        self.TRANSLATE_SCALING = 0.25
        self.ROTATE_SCALING = 0.5

    def joy_callback(self, joy_msg):
        twist = Twist()

        # Initialize inputs
        forward_input = 0.0
        sideways_input = 0.0
        direction_reversed = False

        # Check if Axis 5 (right trigger) is fully pressed (close to -1.0)
        dead_man_switch = joy_msg.axes[5] <= -0.95

        if dead_man_switch:
            # Axis 1 (left stick Y-axis) for forward/backward movement
            twist.linear.x = joy_msg.axes[1] * self.TRANSLATE_SCALING
            # Axis 2 (right stick X-axis) for left/right movement
            sideways_input = joy_msg.axes[2] * self.ROTATE_SCALING

            if forward_input < 0:
                twist.angular.z = -1.0 * sideways_input
            else:
                twist.angular.z = sideways_input

        # Display Xbox controller axes data
        self.get_logger().info("Xbox Controller Axes:")
        for i, axis in enumerate(joy_msg.axes):
            self.get_logger().info(f"  Axis {i}: {axis:.2f}")

        # Display Twist output
        self.get_logger().info("Twist Output:")
        self.get_logger().info(f"  Linear X: {twist.linear.x:.2f} m/s")
        self.get_logger().info(f"  Angular Z: {twist.angular.z:.2f} rad/s")
        self.get_logger().info(
            f'  Dead Man\'s Switch: {"Engaged" if dead_man_switch else "Disengaged"}'
        )
        self.get_logger().info(
            f'  Direction Reversed: {"Yes" if direction_reversed else "No"}'
        )

        self.get_logger().info("------------------------")

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    xbox_controller_node = XboxControllerNode()
    rclpy.spin(xbox_controller_node)
    xbox_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
