# A single detection from a vision node.
#
# Combines the image-space result (where the object appears on screen) with the
# optional 3D result (where it is in space). Both are expressed in the frame given by
# the containing DetectionArray's header, so one TF lookup moves the whole detection
# into whatever frame a consumer works in.
#
# Why not vision_msgs/Detection2D: its BoundingBox2D is a centre point plus a size and
# a rotation, which describes a rotated rectangle. An ArUco marker seen at an angle
# projects to a general quadrilateral, which a rotated rectangle cannot represent, so
# the bounding box below is a polygon instead.

# Bounding box as a closed polygon of corner points (image pixels).
# Axis-aligned detections = 4 corners; oriented / ArUco = the perspective quad.
# The polygon is implicitly closed, so 4 points draw 4 connected edges.
geometry_msgs/Polygon bounding_box

# Colour used to render the bounding box and label in the overlay node.
# RGBA components in [0.0, 1.0]. Alpha 1.0 = fully opaque.
std_msgs/ColorRGBA color

# Human-readable class label (e.g. "aruco_12", "cube_blue", "cube_green").
string class_label

# Numeric identity of the detection: the ArUco marker ID, or the cube class index.
# Provided so consumers do not have to parse class_label.
int32 id

# Confidence score (0.0 to 1.0) indicating the model's certainty.
float32 confidence

# Whether `pose` holds a usable 3D result.
#
# False means the object was seen in the image but could not be located in space -
# no depth return, no camera calibration, or a detector running in 2D-only mode.
# This is reported rather than dropped, because "I can see it but cannot place it"
# is useful to a consumer, whereas silence is not.
bool has_pose

# 3D pose of the detection, in the frame given by DetectionArray.header.frame_id
# (the camera optical frame). Only meaningful when has_pose is true.
#
# A publisher deriving the pose from a second sensor, such as a depth image, must use a
# stream registered to the image the polygon came from, so that one frame_id is correct
# for both halves of this message.
# Consumers transform this into odom/map themselves via tf2, which lets them choose
# the lookup time rather than inheriting the detector's.
geometry_msgs/Pose pose

# Source image crop that generated this detection (e.g. region proposal).
# Useful for logging and offline debugging. Leave empty in normal operation
# to avoid embedding a full JPEG in every detection message.
sensor_msgs/CompressedImage source_image
