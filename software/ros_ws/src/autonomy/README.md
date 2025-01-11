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
