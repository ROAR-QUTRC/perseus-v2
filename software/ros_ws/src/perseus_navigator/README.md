# perseus_navigator package

## Structure

```
.
├── behavior_trees
├── params
├── launch
├── include
└── src
```

Joint publishing (test) :

```
ros2 run tf2_ros static_transform_publisher 1 2 0 0 0 0 1 odom base_link
ros2 run tf2_ros static_transform_publisher 1 2 0 0 0 0 1 map odom
```
