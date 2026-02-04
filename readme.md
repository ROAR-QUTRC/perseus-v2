# nix-ros-overlay ament-cmake-vendor-package Fix

This document describes the issue that was affecting our Nix-based ROS builds and the upstream fix that resolved it.

## The Problem

### Issue Summary

When using `buildEnv` (via `buildROSWorkspace` from `nix-ros-workspace` or similar), both `ament-cmake-vendor-package` and `ament-cmake-vendor-package-wrapped` could end up in the paths list, causing a build failure with the following error:

```
pkgs.buildEnv error: two given paths contain a conflicting subpath:

  `/nix/store/...-ros-jazzy-ament-cmake-vendor-package-2.5.4-r1/share/ament_cmake_vendor_package/local_setup.sh' and

  `/nix/store/...-ros-jazzy-ament-cmake-vendor-package-2.5.4-r1/share/ament_cmake_vendor_package/local_setup.sh'
```

### Root Cause

In `nix-ros-overlay`'s `ros2-overlay.nix`, two versions of the package were defined:

1. **`ament-cmake-vendor-package`** - The base version with path substitutions
2. **`ament-cmake-vendor-package-wrapped`** - An extended version that includes `ament_vendor_wrapper.cmake`

Packages using `patchAmentVendor*` functions depend on the `-wrapped` version, while other packages depend on the base version. When a devShell or `buildEnv` includes both types of packages, both versions end up in the closure and conflict because they produce files at the same paths.

### Previous Workaround

Setting `ignoreCollisions = true` on the `buildEnv` worked since the conflicting files were functionally identical, but this was not an ideal solution as it could mask other legitimate conflicts.

## The Fix

### Upstream PR

- **PR**: [lopsided98/nix-ros-overlay#781](https://github.com/lopsided98/nix-ros-overlay/pull/781)
- **Issue**: [lopsided98/nix-ros-overlay#776](https://github.com/lopsided98/nix-ros-overlay/issues/776)
- **Author**: Michal Sojka (@wentasah)
- **Status**: Merged

### Solution

The fix unifies both packages into a single `ament-cmake-vendor-package` derivation. The unified package now behaves dynamically based on how it's invoked:

- If `cmake` is invoked with any of the understood `-DAMENT_VENDOR_NIX_*` flags, the package behaves as the previously available `ament-cmake-vendor-package-wrapped`
- Otherwise, it behaves as the upstream `ament-cmake-vendor-package`

This ensures only one version exists in the dependency graph, eliminating the collision entirely.

### Files Changed

The PR modified three files in `nix-ros-overlay`:

1. `distros/ament_vendor_wrapper.cmake` - Added conditional logic for NIX flags
2. `distros/ros2-overlay.nix` - Removed the separate `-wrapped` derivation
3. `lib/default.nix` - Simplified vendoring functions

## Verification

This fix was tested in the `bugfix/nix-overlay-test` branch by:

1. Updating `flake.lock` to use nix-ros-overlay revision `3d05d46` (Feb 3, 2026)
2. Successfully building the full ROAR workspace with `nix build .#default`

The build completed without any `buildEnv` collision errors, confirming the fix works correctly.

## References

- Upstream Issue: https://github.com/lopsided98/nix-ros-overlay/issues/776
- Upstream PR: https://github.com/lopsided98/nix-ros-overlay/pull/781
- nix-ros-overlay: https://github.com/lopsided98/nix-ros-overlay
- nix-ros-workspace: https://github.com/RandomSpaceship/nix-ros-workspace
