import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    """Republishes an unstamped Twist on cmd_vel_in as a TwistStamped on cmd_vel_out."""

    def __init__(self):
        super().__init__("twist_stamper")
        self._pub = self.create_publisher(TwistStamped, "cmd_vel_out", 10)
        self._sub = self.create_subscription(Twist, "cmd_vel_in", self._on_twist, 10)

    def _on_twist(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = msg
        self._pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
