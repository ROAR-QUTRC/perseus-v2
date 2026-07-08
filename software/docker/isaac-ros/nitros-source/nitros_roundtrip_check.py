"""
Stage 3 round-trip verification: publish std_msgs/Empty on
/topic_forward_input, wait for it to arrive on
/final/topic_forward_output after passing through two chained
NitrosEmptyForwardNode composable nodes. Our own minimal code, not NVIDIA's
-- see nitros_roundtrip_launch.py.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class RoundtripCheck(Node):
    def __init__(self):
        super().__init__('nitros_roundtrip_check')
        self.received = 0
        self.sub = self.create_subscription(
            Empty, 'final/topic_forward_output', self.on_msg, 10)
        self.pub = self.create_publisher(Empty, 'topic_forward_input', 10)

    def on_msg(self, _msg):
        self.received += 1


def main():
    rclpy.init()
    node = RoundtripCheck()
    end_time = time.time() + 15
    sent = 0
    while time.time() < end_time and node.received == 0:
        sent += 1
        node.pub.publish(Empty())
        rclpy.spin_once(node, timeout_sec=0.2)

    node.get_logger().info(f'sent={sent} received={node.received}')
    ok = node.received > 0
    print('ROUNDTRIP OK' if ok else 'ROUNDTRIP FAILED')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
