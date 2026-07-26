# All 2D detections from a single camera frame.
# header.stamp/frame_id should match the source image so overlay/consumers can
# sync detections to the right frame.
std_msgs/Header header
Detection2D[] detections
