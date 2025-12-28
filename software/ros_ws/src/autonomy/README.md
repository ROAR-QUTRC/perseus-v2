## Autonomy Package

To run without actual Perseus hardware set use_mock_hardware to true when launching:

```
ros2 launch autonomy mapping_using_slam_toolbox.launch.py use_mock_hardware:=true
```

This ROS2 package is intended to cover functionality to:

- create maps of the locally experienced environment from sensor data (lidar and depth cameras)
- determine navigability of the map for large rovers
- localisation of the rover in its known map
- support self-driving to human-selected waypoints
- implement self-driving exploration routines to explore the local area
- implement self-correction activities for loss of network events
- create heatmaps of network connectivity for display to operators
- monitor and generate position related status events (e.g. lost traction and bogged"

### Dependencies

SLAM toolbox
Nav2


### Navigate through multiple waypoints
```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
"{poses: [
  {header: {frame_id: map}, pose: {position: {x: 11.5616, y: 14.1272, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.9959, w: 0.0913}}},
  {header: {frame_id: map}, pose: {position: {x: 0.7224,  y: 6.4172,  z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.5697, w: 0.8218}}},
  {header: {frame_id: map}, pose: {position: {x: 2.7241,  y: 2.5618,  z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.3337, w: 0.9427}}},
  {header: {frame_id: map}, pose: {position: {x: -0.8341, y: -7.2169, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.9812, w: 0.1931}}},
  {header: {frame_id: map}, pose: {position: {x: 11.7310, y: -5.7317, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9419, w: 0.3358}}}
]}"
```


```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
"{poses: [
  {header: {frame_id: map}, pose: {position: {x: 11.5616, y: 14.1272, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.9959, w: 0.0913}}},
  {header: {frame_id: map}, pose: {position: {x: 0.7224,  y: 6.4172,  z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.5697, w: 0.8218}}},
  {header: {frame_id: map}, pose: {position: {x: 2.7241,  y: 2.5618,  z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.3337, w: 0.9427}}},
  {header: {frame_id: map}, pose: {position: {x: -0.8341, y: -7.2169, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.9812, w: 0.1931}}},
  {header: {frame_id: map}, pose: {position: {x: 11.7310, y: -5.7317, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9419, w: 0.3358}}}
]}"
```


```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
"{poses: [
  {header: {frame_id: map}, pose: {position: {x: 0.0,  y: 0.0,  z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}},
  {header: {frame_id: map}, pose: {position: {x: 0.7224,  y: 6.4172,  z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  {header: {frame_id: map}, pose: {position: {x: 2.7241,  y: 2.5618,  z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  {header: {frame_id: map}, pose: {position: {x: -0.8341, y: -7.2169, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  {header: {frame_id: map}, pose: {position: {x: 11.7310, y: -5.7317, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
]}"
```

### Individual Waypoints Reference

**Waypoint 1:**
- Position: x: 11.5616, y: 14.1272, z: 0.0
- Orientation: x: 0.0, y: 0.0, z: 0.9959, w: 0.0913

**Waypoint 2:**
- Position: x: 0.7224, y: 6.4172, z: 0.0
- Orientation: x: 0.0, y: 0.0, z: 0.5697, w: 0.8218

**Waypoint 3:**
- Position: x: 2.7241, y: 2.5618, z: 0.0
- Orientation: x: 0.0, y: 0.0, z: 0.3337, w: 0.9427

**Waypoint 4:**
- Position: x: -0.8341, y: -7.2169, z: 0.0
- Orientation: x: 0.0, y: 0.0, z: 0.9812, w: 0.1931

**Waypoint 5:**
- Position: x: 11.7310, y: -5.7317, z: 0.0
- Orientation: x: 0.0, y: 0.0, z: -0.9419, w: 0.3358

### Single Pose Commands

#### WP1:
```bash
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 11.5616, y: 14.1272, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.9959, w: 0.0913}}}}"
```

#### WP2:
```bash
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 0.7224, y: 6.4172, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.5697, w: 0.8218}}}}"
```

#### WP3:
```bash
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 2.7241, y: 2.5618, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.3337, w: 0.9427}}}}"
```

#### WP4:
```bash
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: -0.8341, y: -7.2169, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.9812, w: 0.1931}}}}"
```

#### WP5:
```bash
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 11.7310, y: -5.7317, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9419, w: 0.3358}}}}"
```


