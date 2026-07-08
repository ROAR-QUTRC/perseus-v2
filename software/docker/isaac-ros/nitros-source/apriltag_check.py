"""
Stage 4 AprilTag ground-truth check: publish NVIDIA's own test image
(isaac_ros_apriltag/test/test_cases/apriltag0/image.png, a known tag36h11
id=0 tag) + matching CameraInfo, run it through our self-built
nvidia::isaac_ros::apriltag::AprilTagNode, and compare the detection against
the exact expected values from NVIDIA's own isaac_ros_apriltag_pol_test.py
(corners/center/pose) -- without needing isaac_ros_test (which pulls in
torch transitively for an unrelated helper). Our own minimal code.
"""
import json
import pathlib
import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

TEST_DIR = pathlib.Path(__file__).parent / 'isaac_ros_apriltag' / \
    'isaac_ros_apriltag' / 'test' / 'test_cases' / 'apriltag0'

# Ground truth from NVIDIA's isaac_ros_apriltag_pol_test.py
EXPECTED_CENTER = (926.0, 547.0)
EXPECTED_CORNERS = [(1044.0, 665.0), (808.0, 665.0), (808.0, 429.0), (1044.0, 429.0)]
DELTA = 2.0


def load_camera_info(path):
    d = json.loads(path.read_text())
    msg = CameraInfo()
    msg.header.frame_id = d['header']['frame_id']
    msg.width = d['width']
    msg.height = d['height']
    msg.distortion_model = d['distortion_model']
    msg.d = d['D']
    msg.k = d['K']
    msg.r = d['R']
    msg.p = d['P']
    return msg


class AprilTagCheck(Node):
    def __init__(self):
        super().__init__('apriltag_check')
        self.received = None
        self.sub = self.create_subscription(
            AprilTagDetectionArray, 'tag_detections', self.on_detections, 10)
        self.image_pub = self.create_publisher(Image, 'image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

    def on_detections(self, msg):
        if self.received is None:
            self.received = msg


def main():
    image_json = json.loads((TEST_DIR / 'image.json').read_text())
    cv_image = cv2.imread(str(TEST_DIR / image_json['image']))
    bridge = CvBridge()
    image_msg = bridge.cv2_to_imgmsg(cv_image, encoding=image_json['encoding'])
    camera_info_msg = load_camera_info(TEST_DIR / 'camera_info.json')

    rclpy.init()
    node = AprilTagCheck()

    end_time = time.time() + 20
    sent = 0
    while time.time() < end_time and node.received is None:
        sent += 1
        camera_info_msg.header.stamp = node.get_clock().now().to_msg()
        image_msg.header.stamp = camera_info_msg.header.stamp
        node.camera_info_pub.publish(camera_info_msg)
        node.image_pub.publish(image_msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(f'sent={sent} received={node.received is not None}')

    ok = node.received is not None and len(node.received.detections) >= 1
    if ok:
        det = node.received.detections[0]
        ok = (det.id == 0 and det.family == 'tag36h11'
              and abs(det.center.x - EXPECTED_CENTER[0]) <= DELTA
              and abs(det.center.y - EXPECTED_CENTER[1]) <= DELTA)
        for i, (ex, ey) in enumerate(EXPECTED_CORNERS):
            ok = ok and abs(det.corners[i].x - ex) <= DELTA \
                and abs(det.corners[i].y - ey) <= DELTA
        print(f'id={det.id} family={det.family} '
              f'center=({det.center.x:.1f},{det.center.y:.1f}) '
              f'expected_center={EXPECTED_CENTER}')

    print('APRILTAG OK' if ok else 'APRILTAG FAILED')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
