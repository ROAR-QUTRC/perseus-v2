# A single 2D detection result from an ML model.
#
# Represents one detected object in image space: a bounding box,
# the predicted class label, and the model's confidence score.
# Detections are published as Detection2DArray (one message per frame).
#
# Why not vision_msgs/Detection2D: its BoundingBox2D is a centre point plus a
# size and a rotation, which describes a rotated rectangle. An ArUco marker seen
# at an angle projects to a general quadrilateral, which a rotated rectangle
# cannot represent, so the bounding box below is a polygon instead.

# Bounding box as a closed polygon of corner points (image pixels).
# Axis-aligned detections = 4 corners; oriented / ArUco = the perspective quad.
# The polygon is implicitly closed, so 4 points draw 4 connected edges.
geometry_msgs/Polygon bounding_box

# Colour used to render the bounding box and label in the overlay node.
# RGBA components in [0.0, 1.0]. Alpha 1.0 = fully opaque.
std_msgs/ColorRGBA color

# Human-readable class label (e.g. "aruco_12", "cube_blue", "cube_green").
string class_label

# Confidence score (0.0 to 1.0) indicating the model's certainty.
float32 confidence

# Source image crop that generated this detection (e.g. region proposal).
# Useful for logging and offline debugging. Leave empty in normal operation
# to avoid embedding a full JPEG in every detection message.
sensor_msgs/CompressedImage source_image
