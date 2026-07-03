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

### `gz sim` segfaults from a contaminated `GZ_CONFIG_PATH` — 2026-07-03

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `software/ros_ws/src/perseus_lite_simulation/launch/gazebo.launch.py`
- **Pattern:** `gz sim`'s core library and its plugins (rendering, physics, sensors) are resolved separately: the core binary comes from whatever's first on `PATH`/`LD_LIBRARY_PATH`, but which _plugin_ `.so` files it `dlopen`s is governed entirely by `GZ_CONFIG_PATH`. If a machine also has a native/system ROS 2 install (e.g. `/opt/ros/jazzy` sourced unconditionally from `~/.bashrc`), that install's own `setup.bash` sets `GZ_CONFIG_PATH` to its own Gazebo vendor packages' config dirs. Pixi's `simulation` env activation does not clear or override this var, so it leaks through from the parent shell. Result: the Pixi env's `gz-sim` core (v8.10.0 here) loads plugins built for the system install's different version (v8.11.0) — an ABI mismatch that segfaults (exit 139) immediately on rendering-engine plugin load, with **zero log output** (no Python traceback, no rclpy error, nothing — the crash is below the level anything in-process can catch or print).
- **Root cause:** Pixi/conda environment activation only adds/prepends env vars on top of whatever the parent shell already exported; it doesn't reset tool-specific vars like `GZ_CONFIG_PATH` that a _different_ toolchain's activation script (system ROS's `setup.bash`) had already set in that same shell.
- **First (wrong) diagnosis:** Initially misattributed this to the user's account not being in the `render` Unix group (needed for `/dev/dri/renderD128` GPU access). That group membership is real and worth having, but adding it did **not** fix the segfault — confirmed by reproducing the identical crash with `render` group active. The actual fix (below) was found by dumping the full env inside the activated Pixi shell and noticing `GZ_CONFIG_PATH` pointed entirely at `/opt/ros/jazzy`.
- **Fix applied:** `gazebo.launch.py`'s `additional_env` now explicitly sets `GZ_CONFIG_PATH` to `$CONDA_PREFIX/share/gz` (the Pixi env's own Gazebo config directory), overriding whatever the parent shell inherited. Confirmed fix by clearing/repointing the var manually first (`unset` and `$CONDA_PREFIX/share/gz` both avoided the crash; `""` did not — gz-sim treats an empty string as "no config" rather than falling back to a compiled-in default, so it must be pointed at a real directory, not just cleared).
- **Prevention rule:** When a Pixi/conda-activated tool crashes or misbehaves in a way that doesn't match its own version, dump the full environment (`env | grep -i <tool>`) inside the activated shell and check for the _other_ installation's paths (system package manager installs, a different environment manager, etc.) bleeding through — conda/Pixi activation is additive, not isolating, for vars it doesn't itself manage. Don't stop at the first plausible-sounding cause (like a permissions issue) without reproducing the fix in isolation first.
