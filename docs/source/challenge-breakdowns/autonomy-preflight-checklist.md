# Pre-Flight Checklist

Use this checklist before every autonomous run.

## TF Verification

- [ ] `ros2 run tf2_tools view_frames` — Tree is complete and connected
- [ ] `ros2 run tf2_ros tf2_echo odom base_link` — Transform exists
- [ ] `ros2 run tf2_ros tf2_echo map odom` — Transform exists (SLAM/AMCL running)
- [ ] No broken links or missing frames in TF tree

## Odometry Verification

- [ ] `ros2 topic echo /odom --once` — Data present, values sensible
- [ ] `ros2 topic hz /odom` — Rate ~50 Hz
- [ ] Drive forward manually — X position increases
- [ ] Turn left manually — Yaw increases (CCW positive)
- [ ] Velocities in `/odom` match commanded direction
- [ ] No sudden jumps or discontinuities

## Sensor Verification

### LiDAR

- [ ] `ros2 topic echo /scan --once` — Data present
- [ ] `ros2 topic hz /scan` — Rate ~10 Hz
- [ ] Scan range values are reasonable (not all zeros or max)
- [ ] LiDAR visible in RViz, aligned with robot

### IMU

- [ ] `ros2 topic echo /imu/data --once` — Data present
- [ ] `ros2 topic hz /imu/data` — Rate ~100 Hz
- [ ] Orientation quaternion is valid (magnitude ≈ 1)
- [ ] Robot stationary shows ~9.81 m/s² on Z acceleration

## EKF Verification

- [ ] `ros2 topic echo /odometry/filtered --once` — Data present
- [ ] `ros2 topic hz /odometry/filtered` — Rate ~30 Hz
- [ ] Filtered odometry tracks raw odometry (no divergence)
- [ ] No EKF rejection warnings in logs

## Navigation Verification

- [ ] All lifecycle nodes active:
  ```console
  ros2 lifecycle list /bt_navigator
  ros2 lifecycle list /controller_server
  ros2 lifecycle list /planner_server
  ```
- [ ] Costmaps visible in RViz
- [ ] Local costmap shows obstacles correctly
- [ ] Global costmap loaded (if using pre-built map)
- [ ] Send test goal via RViz "2D Goal Pose"
- [ ] Robot moves toward goal without collision
- [ ] Path is planned and displayed in RViz

## Safety Verification

- [ ] Emergency stop tested and functional
- [ ] E-STOP physically accessible
- [ ] Collision monitor active: `ros2 topic echo /collision_monitor_state`
- [ ] Manual override via teleop works
- [ ] Twist mux switches correctly between nav and teleop

## Environment Verification

- [ ] Area clear of unexpected obstacles
- [ ] Sufficient lighting for camera (if used)
- [ ] Network connection stable
- [ ] Battery level adequate for planned operation

## Competition-Specific

- [ ] Start position marked and known
- [ ] Goal waypoints programmed (if applicable)
- [ ] Map of arena loaded (if pre-mapping allowed)
- [ ] Recording started: `ros2 bag record ...`

---

## Quick Verification Commands

```console
# All-in-one TF check
ros2 run tf2_tools view_frames && xdg-open frames.pdf

# Topic rates summary
ros2 topic hz /scan /odom /imu/data /odometry/filtered

# Lifecycle status
ros2 lifecycle list /bt_navigator

# System diagnostics
ros2 doctor

# Node graph
ros2 run rqt_graph rqt_graph
```

---

## If Something Fails

| Check Failed     | First Action                                   |
| ---------------- | ---------------------------------------------- |
| TF missing       | Verify launch order, check RSP is running      |
| No odometry      | Check controller manager, joint states         |
| No LiDAR data    | Verify IP/port, check network connection       |
| No IMU data      | Check I2C connection, verify device address    |
| EKF diverging    | Check sensor covariances, rejection thresholds |
| Nav2 won't start | Check all TF frames exist, verify map loaded   |
| Robot won't move | Check E-STOP, verify `/cmd_vel` is published   |

See [Troubleshooting](autonomy-troubleshooting.md) for detailed solutions.
