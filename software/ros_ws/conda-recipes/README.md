# Conda recipes — packages missing from RoboStack

The Pixi migration (see top-level `pixi.toml`) gets ROS 2 Jazzy and essentially
every dependency from the `robostack-jazzy` conda channel. This directory is for
the few packages that are **not** in RoboStack and would need a conda recipe
(one subdir per package, each a `recipe.yaml` for `rattler-build`).

```bash
rattler-build build --recipe conda-recipes/<pkg>/recipe.yaml --output-dir ./local-channel
# then in pixi.toml: channels = ["./local-channel", "robostack-jazzy", "conda-forge"]
```

## Status: almost nothing is actually needed

The original Nix build carried a set of upstream third-party packages
(`lidarslam_ros2`, `ndt-omp-ros2`, `opennav-coverage`, `fields2cover`). These
were **dropped** during the migration — **no package, launch file, or config in
`software/ros_ws/src/` references them**, so the lite robot never used them.
They are not ported. If a future feature needs one, write its recipe here.

## Remaining optional items

| Package         | Why deferred                                                                                                                                             | Action                                                                                                                                                       |
| --------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `twist_stamper` | Not in robostack-jazzy. A `perseus_lite_simulation` **runtime** (`exec_depend`) — sim _builds_ without it; only launching needs it.                      | Vendor the tiny upstream ament_python package into `software/ros_ws/src/`, or write a recipe. (Auto-cloning was blocked as untrusted; do this deliberately.) |
| `groot2`        | No conda package (BehaviorTree GUI debug tool).                                                                                                          | Install the v1.6.1 AppImage manually, or add a `pixi run install-groot2` task that downloads it. Low priority.                                               |
| RealSense       | `realsense2-camera` pulls a conflicting `ros2-distro-mutex`/`libboost`/`libudev` set (breaks the solve, esp. linux-aarch64). Optional on the lite robot. | If needed, add as a separate linux-64 optional environment so it can't poison the default solve.                                                             |

## Already available — no recipe needed

- **Open3D** — on conda-forge (`open3d` 0.19.0). Add to a feature's deps if used.
- The `software/shared/{crc,fd-wrapper,ptr-wrapper,simple-networking,type-demangle}`
  C++ libs build **in-tree**: `perseus_sensors`'s CMake `add_subdirectory`
  fallback compiles them. No recipe, no package.xml.
- All core ROS 2, Nav2, slam*toolbox, robot_localization, ros2_control,
  behaviortree_cpp, ros_gz*\*, rviz2, rplidar_ros, twist_mux — in robostack-jazzy.
