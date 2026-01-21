# Troubleshooting and Reference

## Common Failure Modes

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Robot turns opposite direction | Frame sign mismatch or encoder direction | Check wheel joint directions in URDF |
| Nav2 won't activate | Missing TF or lifecycle failure | `ros2 run tf2_tools view_frames`, check lifecycle states |
| SLAM/AMCL diverging | Wheel radius or encoder misconfiguration | Verify `wheel_radius` and `wheel_separation` |
| Costmap shows phantom obstacles | Sensor frame mismatch | Verify `laser_2d_frame` position in URDF |
| Robot oscillates | Tuning too aggressive | Reduce `max_vel_*` and `acc_lim_*` |
| Localisation jumps | AMCL particle depletion | Increase `min_particles` |
| `/odometry/filtered` lags | EKF rejecting measurements | Check rejection thresholds in `ekf_params.yaml` |
| Robot doesn't move | E-STOP engaged or no cmd_vel | Check E-STOP, verify topic flow |
| Path planning fails | Costmap inflation too large | Reduce inflation radius |
| Robot stops at obstacles | Collision monitor too sensitive | Adjust `PolygonStop` radius |

## Debug Commands

### TF Debugging

```console
# Generate TF tree PDF
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link

# Monitor transform updates
ros2 run tf2_ros tf2_monitor

# List all frames
ros2 run tf2_ros tf2_echo --all
```

### Topic Debugging

```console
# Monitor topic rates
ros2 topic hz /scan /odom /imu/data

# Check topic info
ros2 topic info /scan -v

# Echo with timestamp
ros2 topic echo /odom --field header.stamp

# Count messages
ros2 topic echo /scan --once | wc -l
```

### Node Debugging

```console
# List all nodes
ros2 node list

# Node info
ros2 node info /bt_navigator

# Check lifecycle state
ros2 lifecycle get /controller_server

# Transition lifecycle
ros2 lifecycle set /bt_navigator activate
```

### System Diagnostics

```console
# ROS2 system check
ros2 doctor

# View computation graph
ros2 run rqt_graph rqt_graph

# Parameter inspection
ros2 param list /controller_server
ros2 param get /controller_server robot_radius
```

### Nav2 Specific

```console
# Check behaviour tree status
ros2 topic echo /bt_navigator/transition_event

# View current goal
ros2 topic echo /goal_pose

# Check costmap
ros2 topic echo /local_costmap/costmap --once

# Cancel navigation
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --cancel
```

---

## Change Control

### Before Changing Any Contract Item

1. **Open an issue** in the repository describing the proposed change
2. **Notify the autonomy team** via the issue
3. **Document the rationale** and expected impact
4. **Update the documentation** with the new values
5. **Run the validation procedure** (pre-flight checklist)
6. **Tag a known-good configuration** before merging

### Versioning Strategy

Before major changes, tag the current working configuration:

```console
git tag autonomy-known-good-$(date +%Y%m%d)
git push origin --tags
```

### What Requires Change Control

| Change Type | Requires Coordination |
|-------------|----------------------|
| Joint names | Yes |
| Topic names | Yes |
| Frame names | Yes |
| Wheel geometry | Yes |
| Sensor mounting | Yes |
| Nav2 parameters | Recommended |
| EKF parameters | Recommended |
| Launch file structure | Recommended |

---

## Key Configuration Files

| File | Purpose |
|------|---------|
| `autonomy/config/ekf_params.yaml` | EKF sensor fusion tuning |
| `autonomy/config/slam_toolbox_params.yaml` | SLAM configuration |
| `autonomy/config/perseus_nav_params.yaml` | Nav2 navigation parameters |
| `perseus/config/perseus_controllers.yaml` | DiffDrive controller settings |
| `perseus_description/urdf/perseus.urdf.xacro` | Robot model and geometry |
| `perseus_description/ros2_control/perseus.ros2_control.xacro` | Hardware interface config |
| `perseus_sensors/i2c_imu_driver/config/i2c_imu_config.yaml` | IMU driver settings |

## Known-Good Topic Snapshot

```
/cmd_vel                geometry_msgs/msg/Twist
/cmd_vel_nav            geometry_msgs/msg/Twist
/imu/data               sensor_msgs/msg/Imu
/joint_states           sensor_msgs/msg/JointState
/map                    nav_msgs/msg/OccupancyGrid
/odom                   nav_msgs/msg/Odometry
/odometry/filtered      nav_msgs/msg/Odometry
/scan                   sensor_msgs/msg/LaserScan
/tf                     tf2_msgs/msg/TFMessage
/tf_static              tf2_msgs/msg/TFMessage
```

## Related Documentation

- [Robot Description](../development/software/robot-description.md) — URDF architecture and frames
- [Local Localisation](../development/software/local-localisation.md) — EKF tuning guide
