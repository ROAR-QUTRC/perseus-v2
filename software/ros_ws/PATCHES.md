# ROS Package Patches and Overrides

This document describes the patches and custom package builds maintained in `patches.nix` for the Perseus project.

## Table of Contents

- [rplidar-ros](#rplidar-ros)
- [Other Patches](#other-patches)

---

## rplidar-ros

### Status

**Custom build from source** (not using rosdistro package)

### Why This Exists

The `rplidar-ros` package is **not officially released for ROS 2 Jazzy** in the rosdistro repository. While it was available for earlier distributions like Humble, it has not been ported to Jazzy as of November 2025.

You can verify this by checking the [official Jazzy distribution file](https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml) - rplidar_ros is not present.

### History

1. **Before June 2025**: The older version of `nix-ros-overlay` (commit `503be406`) may have included rplidar-ros through cross-distro access or manual overrides

2. **June 8, 2025** (commit `3904f9f`): An override was added in `patches.nix` attempting to upgrade what was thought to be an "old version" of rplidar-ros to v2.1.5 to support the C1 sensor

3. **November 19, 2025** (commit `dbd25bf` on branch `feat/flake-update`): When updating `flake.lock` to a newer `nix-ros-overlay` (commit `41e92bed`), the base rplidar-ros package disappeared, causing the override to fail with "package not found" errors

### Current Solution

Since rplidar-ros is not in the official Jazzy rosdistro, we build it as a custom package from the upstream Slamtec repository's `ros2` branch. The implementation in `patches.nix` uses `buildRosPackage` to compile it from source rather than attempting to override a non-existent base package.

### Version Information

- **Version**: 2.1.5
- **Source**: [Slamtec/rplidar_ros](https://github.com/Slamtec/rplidar_ros) (ros2 branch)
- **Notable Features**: Version 2.1.5 adds support for the RPLiDAR C1 sensor
- **License**: BSD-2-Clause

### Dependencies

```nix
buildInputs = [ ament-cmake-ros ]
propagatedBuildInputs = [ rclcpp sensor-msgs std-srvs ]
```

### Usage

The package is included in the Perseus Lite configuration and can be launched via:

```bash
ros2 launch rplidar_ros rplidar_c1_launch.py
```

See `src/perseus_lite/launch/perseus_lite.launch.py` for integration details.

### Future Considerations

If rplidar-ros is officially released to Jazzy rosdistro in the future, this custom build should be:

1. Removed from `patches.nix`
2. Added as a regular dependency in the relevant package.xml files
3. The package will then be automatically included via nix-ros-overlay

To check if it's been added to Jazzy, monitor the [rosdistro jazzy distribution.yaml](https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml) file.

---

## Other Patches

(Documentation for other patches in `patches.nix` can be added here)
