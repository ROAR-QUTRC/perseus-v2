# Message representing multiple object detections
# Time at which the detections are valid (e.g. camera header.stamp)
builtin_interfaces/Time stamp

# Frame in which the poses are expressed (e.g. "odom" or "map")
string frame_id

# IDs of all detected markers
int32[] ids

# Pose of each detected marker, in the same order as `ids`
# (ids[i] corresponds to poses[i]).
# For 2D-only detections where no 3D pose could be estimated, all pose fields
# are set to NaN and consumers must check for finite values before use.
geometry_msgs/Pose[] poses

# 2D image-space region for each detection, in the same order as `ids`
sensor_msgs/RegionOfInterest[] regions_of_interest
