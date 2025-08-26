To build the package:
```sh
nix-shell -p gcc14 onnxruntime --run "colcon build --packages-select perseus_vision"
```

To run the node:
```sh
nix-shell -p gcc14 onnxruntime --run "source install/setup.bash && ros2 run perseus_vision cube_detector"
```