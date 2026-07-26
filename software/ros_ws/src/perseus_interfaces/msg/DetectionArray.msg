# All detections from a single camera frame.
#
# header.stamp/frame_id match the source image, so the bounding polygons are in that
# image's pixel coordinates and any 3D poses are in that camera's optical frame.
std_msgs/Header header
Detection[] detections
