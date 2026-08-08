# Result of comparing commanded velocity on /cmd_vel_out against velocity measured on
# /odometry/filtered, produced by watchdog/mobility_watchdog.
#
# The filtered odometry has no wheel-encoder input (see autonomy_bringup/config/
# ekf_config.yaml) -- it is derived from FAST-LIO's LiDAR-inertial pose and the IMU gyro --
# so a sustained gap between command and measurement reflects the wheels genuinely failing
# to move the robot, such as when they lose traction in sand, rather than two wheel-derived
# signals agreeing with each other for the wrong reason.

std_msgs/Header header

# Whether the linear or angular channel has enough sustained command history to report a
# fresh assessment this cycle. False while /cmd_vel_out is stale or the robot is
# essentially idle, in which case the fields below hold stale values from the last time
# evaluation ran and must not be treated as a fresh read of the current terrain.
bool is_evaluating

# Whether sustained slip was detected on the linear or angular channel this cycle.
# Always false while is_evaluating is false.
bool is_slipping

# Mean slip ratio over the detection window for forward speed: 0 means commanded speed
# was fully achieved, 1 means no measured forward motion at all despite the command. Not
# clamped, so it can go negative when the robot exceeds the commanded speed (e.g. rolling
# downhill).
float64 linear_slip_ratio

# Same ratio computed from yaw rate instead of forward speed. Generally more trustworthy
# than the linear ratio: the measured value here comes straight from the IMU gyro, whereas
# the measured linear speed is inferred by the EKF from LiDAR pose differencing.
float64 angular_slip_ratio

# Signed commanded and measured values actually compared this cycle, for a consumer that
# wants to apply its own thresholding rather than trusting is_slipping.
float64 commanded_linear_velocity
float64 measured_linear_velocity
float64 commanded_angular_velocity
float64 measured_angular_velocity
