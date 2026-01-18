# SLAM Toolbox Setup for Perseus Lite

## Quick Start

### On the Robot (Jetson Orin Nano)

```bash
cd ~/perseus-v2/software/ros_ws

# Enter nix environment
nix develop

# Build local packages (skip problematic ones)
colcon build --symlink-install --packages-skip perseus_can_if perseus_simulation

# Source the workspace
source install/setup.bash

# Launch SLAM + Nav2
ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py
```

### On the Laptop

```bash
cd ~/perseus-v2/software/ros_ws
nix develop
source install/setup.bash

# Verify connection (should see robot topics)
ros2 topic list

# Launch RViz
rviz2
# Or with GPU support:
nixgl-script rviz2
```

## Troubleshooting

### If Nav2 shows "Waiting for transform from base_link to map"

SLAM Toolbox may not have started properly. The launch file now automatically handles lifecycle transitions with proper timing (waits 2 seconds for the node to initialize before configuring). If issues persist:

```bash
# Check if SLAM Toolbox is running
ros2 node list | grep slam

# Check its lifecycle state
ros2 lifecycle get /slam_toolbox
```

If it still shows `unconfigured`, you can manually configure and activate:

```bash
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

### Verify SLAM is Working

```bash
# Check for map->odom transform
ros2 run tf2_ros tf2_echo map odom

# Check scan data is flowing
ros2 topic hz /scan

# Check the map is being published
ros2 topic hz /map
```

### Key Topics to Monitor

| Topic                | Description                    |
| -------------------- | ------------------------------ |
| `/scan`              | LIDAR data (from rplidar_node) |
| `/map`               | SLAM-generated occupancy grid  |
| `/odom`              | Odometry from wheel encoders   |
| `/odometry/filtered` | Fused odometry from EKF        |
| `/tf`                | Transform tree                 |

### RViz Displays to Add

1. **Map** - Topic: `/map`
2. **LaserScan** - Topic: `/scan`
3. **TF** - Show transforms
4. **RobotModel** - Visualize robot
5. **Path** - Topic: `/plan` (for Nav2 paths)

## Configuration Files

- SLAM params: `src/autonomy/config/slam_toolbox_params.yaml`
- Nav2 params: `src/autonomy/config/perseus_nav_params.yaml`
- EKF params: `src/autonomy/config/ekf_params.yaml`

## Frame Tree

```
map
 └── odom (published by SLAM Toolbox)
      └── base_link (published by EKF/odometry)
           └── chassis
                ├── c1_lidar_frame (RPLIDAR C1)
                ├── imu_link
                ├── left_rocker
                │    ├── front_left_motor → front_left_wheel
                │    └── rear_left_motor → rear_left_wheel
                └── right_rocker
                     ├── front_right_motor → front_right_wheel
                     └── rear_right_motor → rear_right_wheel
```

## Notes

- ROS_DOMAIN_ID is set to 51 for dev environment (automatic in nix shell)
- CUDA-accelerated SLAM is enabled (`use_cuda: true` in slam_toolbox_params.yaml)
- Uses patched slam_toolbox from DingoOz/slam_toolbox fork with null pointer fix
- Both machines must be on the same network
