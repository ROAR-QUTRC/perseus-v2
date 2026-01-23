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


```sh
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
"{poses: [
    {header: {frame_id: map}, pose: {position: {x: 1.8431, y: 1.0241, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.408, w: 0.913}}},
    {header: {frame_id: map}, pose: {position: {x: 2.9887, y: 5.2331, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.841, w: 0.541}}},
    {header: {frame_id: map}, pose: {position: {x: 12.5817, y: 12.9668, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.973, w: 0.231}}},
    {header: {frame_id: map}, pose: {position: {x: 1.6436, y: -7.1337, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.998, w: 0.054}}},
    {header: {frame_id: map}, pose: {position: {x: 12.1729, y: -3.1052, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.841, w: 0.541}}}
]}"
```