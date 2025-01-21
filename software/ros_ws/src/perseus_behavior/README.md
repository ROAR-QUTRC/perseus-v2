# perseus_behavior package

## Structure

```
.
├── behavior_trees
├── config
├── launch
└── src
```

## Tree Executor Example

```
ros2 action send_goal /behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: MainTree}"
```
