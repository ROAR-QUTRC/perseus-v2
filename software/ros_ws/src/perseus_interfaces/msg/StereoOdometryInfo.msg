# Diagnostic information about one stereo_odometry motion estimation pass.
#
# Published alongside the odometry/pose/TF output so a consumer can tell a confident
# estimate from a degraded one without inferring it from the pose alone.

std_msgs/Header header

# True if this pass could not match enough features to estimate motion, so the visual
# odometer was restarted and the published motion for this pass is the identity.
bool got_lost

# True if the next pass will keep the current frame as the matching reference instead
# of advancing to the new one. Set when the estimated motion is small (see
# ref_frame_change_method: 1) or inlier support is low (method: 2), so slow motion
# does not starve the matcher of parallax between frames.
bool change_reference_frame

# False when estimateMotion() itself failed this pass (distinct from got_lost, which
# additionally requires being unable to feed the odometer at all).
bool motion_estimate_valid

int32 num_matches
int32 num_inliers

# Wall-clock time spent processing this image pair, in seconds.
float64 runtime_s
