# Error Log

### Deprecated `tests_require=` in ament_python setup.py — 2026-07-03

- **Severity:** Low
- **Category:** Convention
- **File(s):** `software/ros_ws/src/input_devices/setup.py`, `software/ros_ws/src/teleop_diagnostics/setup.py`
- **Pattern:** Declaring test dependencies via `tests_require=[...]` in a `setup.py`'s `setup()` call. Setuptools deprecated and then dropped support for this argument; under setuptools >=68 it triggers `UserWarning: Unknown distribution option: 'tests_require'` on every build, which colcon surfaces as "N packages had stderr output" even though the build succeeds.
- **Root cause:** These `setup.py` files predate the setuptools deprecation and were never updated when test-dependency declaration moved to the ROS 2 package manifest.
- **Fix applied:** Removed `tests_require=["pytest"],` from both `setup.py` files. Test dependencies are already declared correctly via `<test_depend>pytest</test_depend>` (or equivalent) in each package's `package.xml`, which is the mechanism ament_python/colcon actually uses.
- **Prevention rule:** Never add `tests_require=` to a new ament_python `setup.py`. Declare test dependencies only via `<test_depend>` in `package.xml`. When adding a new Python package, check its `setup.py` doesn't carry this option before merging.

### Launch files still invoking `nix run` after the Nix→Pixi migration — 2026-07-03

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `software/ros_ws/src/perseus_lite_simulation/launch/gazebo.launch.py`, `software/ros_ws/src/perseus_lite_simulation/launch/perseus_sim.launch.py`, `software/ros_ws/src/autonomy/launch/mapping_using_slam_toolbox.launch.py`
- **Pattern:** A `launch.actions.ExecuteProcess` wraps a GUI/rendering binary (Gazebo, RViz) in `nix run --impure github:nix-community/nixGL -- <real cmd>`. This wrapper only made sense under the old Nix dev shell, where Nix-built binaries couldn't see host OpenGL drivers. When a build system migration removes the underlying toolchain (Nix → Pixi/RoboStack here), commit `43c286c2` ("remove Nix build system") deleted `flake.nix` etc. but didn't grep launch files (or any non-toolchain file) for lingering invocations of the old toolchain's CLI.
- **Root cause:** Migration removed the tool the wrapper depended on but not the wrapper itself; nothing failed at build time because `ExecuteProcess` only fails at _launch_ time, so `pixi run -e simulation build` stayed green while `pixi run sim` was broken.
- **Fix applied:** Stripped the 5-token `nix run --impure github:nix-community/nixGL --` prefix and the now-meaningless `NIXPKGS_ALLOW_UNFREE` env var from all three `ExecuteProcess` calls; kept the other `additional_env` entries (`QT_QPA_PLATFORM`, `GZ_SIM_RESOURCE_PATH`, RMW QoS vars) which are still relevant under Pixi.
- **Prevention rule:** After removing a build system/toolchain, `grep -rn` the old tool's invocation string (e.g. `nix run`, `nix develop`) across the _entire_ repo, not just packaging files — launch files, scripts, and CI configs can all embed direct shell-outs to a toolchain that a migration otherwise fully replaces.

### `gz sim` segfaults without GPU render-node group membership — 2026-07-03

- **Severity:** Low
- **Category:** Configuration
- **File(s):** none (local machine, not a repo bug)
- **Pattern:** `gz sim` (Gazebo) needs `/dev/dri/renderD128` for hardware-accelerated (Ogre2) rendering, even in headless server-only mode (`-s`) when the world contains camera sensors. That device is owned by the `render` group; a user account that's only in `video` (sufficient for GLX/X11 apps like `glxinfo`) segfaults (exit 139) the instant `gz sim` tries to initialize rendering.
- **Root cause:** `video` group grants X11/GLX display access but not direct DRM render-node access; the two are commonly conflated since desktop OpenGL apps work fine without `render` membership.
- **Fix applied:** None in-repo — diagnosed via `getent group render` (user absent) and confirmed by forcing `LIBGL_ALWAYS_SOFTWARE=1`, which avoided the crash. Local fix is `sudo usermod -aG render $USER` followed by a re-login (or `newgrp render`) so the new group membership takes effect.
- **Prevention rule:** When a ROS 2 Gazebo sim segfaults immediately on launch with zero log output (not a Python traceback, not an rclpy error), check `groups` for `render` and `ls -la /dev/dri/` for device ownership before assuming a code/config bug — this class of failure is silent at every log level.
