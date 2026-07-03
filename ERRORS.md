# Error Log

### Deprecated `tests_require=` in ament_python setup.py — 2026-07-03

- **Severity:** Low
- **Category:** Convention
- **File(s):** `software/ros_ws/src/input_devices/setup.py`, `software/ros_ws/src/teleop_diagnostics/setup.py`
- **Pattern:** Declaring test dependencies via `tests_require=[...]` in a `setup.py`'s `setup()` call. Setuptools deprecated and then dropped support for this argument; under setuptools >=68 it triggers `UserWarning: Unknown distribution option: 'tests_require'` on every build, which colcon surfaces as "N packages had stderr output" even though the build succeeds.
- **Root cause:** These `setup.py` files predate the setuptools deprecation and were never updated when test-dependency declaration moved to the ROS 2 package manifest.
- **Fix applied:** Removed `tests_require=["pytest"],` from both `setup.py` files. Test dependencies are already declared correctly via `<test_depend>pytest</test_depend>` (or equivalent) in each package's `package.xml`, which is the mechanism ament_python/colcon actually uses.
- **Prevention rule:** Never add `tests_require=` to a new ament_python `setup.py`. Declare test dependencies only via `<test_depend>` in `package.xml`. When adding a new Python package, check its `setup.py` doesn't carry this option before merging.
