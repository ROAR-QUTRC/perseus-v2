To build the package:

```sh
nix-shell -p gcc14 onnxruntime --run "colcon build --packages-select perseus_vision"
```

To run the nodes:

```sh
nix-shell -p gcc14 onnxruntime --run "source install/setup.bash && ros2 launch perseus_vision vision.launch.py"
```

## Special Thanks

Special thanks to the **Monash Rover Team** for pulicing the dataset used to train the YOLO cube detection model.

And Thanks to **ChatGPT, GitHub Colpilot**...
