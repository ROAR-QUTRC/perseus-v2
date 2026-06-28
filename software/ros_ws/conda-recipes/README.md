# Conda recipes — packages missing from RoboStack

The Pixi migration (see top-level `pixi.toml`) gets ROS 2 Jazzy and the vast
majority of dependencies straight from the `robostack-jazzy` conda channel. A
handful of packages the old Nix build provided via
`software/ros_ws/third-party-packages/` + `patches.nix` and `packages/` are **not
in RoboStack** and need conda recipes here, OR vendoring into the colcon
workspace. This directory holds those recipes (one subdir per package, each a
`recipe.yaml` for `rattler-build`).

Build a recipe into a local channel, then point `pixi.toml` at it:

```bash
rattler-build build --recipe conda-recipes/<pkg>/recipe.yaml --output-dir ./local-channel
# then in pixi.toml: channels = ["./local-channel", "robostack-jazzy", "conda-forge"]
```

## Backlog (verified absent from robostack-jazzy on 2026-06-28)

| Package | Source today | Notes |
| --- | --- | --- |
| `ndt-omp-ros2` | `third-party-packages/ndt-omp-ros2` + `patches/ndt-omp-ros2` | Boost patch; move runtime deps to host/run. |
| `lidarslam_ros2` → `scanmatcher`, `graph-based-slam`, `lidarslam`, `lidarslam-msgs` | `third-party-packages/lidarslam-ros2` | 4 sub-packages. graph-based-slam needs nav-msgs/std-srvs; lidarslam-msgs needs ament-cmake-auto; scanmatcher test deps → checkInputs. Depends on `ndt-omp-ros2`. |
| `opennav-coverage` → `opennav-coverage`, `opennav-coverage-navigator`, `opennav-row-coverage` | `third-party-packages/opennav-coverage` | Adds visualization-msgs dep. Depends on `fields2cover`. |
| `fields2cover` | `patches/fields2cover` (built from GitHub) | **Heaviest.** Pulls or-tools, gdal, geos, eigen, swig, tbb, nlohmann_json, tinyxml-2, steering-functions, spline, matplotlib-cpp. All on conda-forge — recipe wires them up. |
| `twist_stamper` | (was robostack in Nix? no — small upstream ament_python pkg) | NOT in robostack-jazzy. Tiny — easiest to **vendor into the colcon workspace** rather than write a recipe. Needed by `perseus_lite_simulation`. |
| `groot2` | `packages/groot2` (AppImage wrapper) | No conda package. Either a recipe wrapping the v1.6.1 AppImage, or a `pixi run install-groot2` task that downloads it. GUI debug tool — low priority. |

## NOT needed as recipes (already available)

- **Open3D** — on conda-forge as `open3d` 0.19.0 (the old custom x86_64 Nix build
  is unnecessary). Add `open3d` to the relevant feature's dependencies.
- All core ROS 2, Nav2, slam_toolbox, robot_localization, ros2_control,
  behaviortree_cpp, ros_gz_*, rviz2, rplidar_ros, twist_mux, etc. — in robostack-jazzy.

## Custom in-workspace C++ libs (NOT conda recipes)

`software/shared/{crc,fd-wrapper,ptr-wrapper,simple-networking,type-demangle}` and
`software/arm-teleop-direct` were built by `software/overlay.nix`. Under Pixi they
should be built **inside the colcon workspace** (give them a minimal
`package.xml`/ament shape, or a top-level CMake superbuild). `perseus_sensors`
depends on `simple-networking`, so that lib must build first.
