# perseus-lite

A 50%-scale, 4-wheel skid-steer rover driven by Feetech ST3215 servos over
serial, with an optional servo manipulator arm. Hard-diverged fork of
[ROAR-QUTRC/perseus-v2](https://github.com/ROAR-QUTRC/perseus-v2).

## Pixi commands

Dev environment and builds are managed by [Pixi](https://pixi.sh). Run
`pixi install` once, then pick an environment and run commands in it with
`pixi run -e <environment> <command>` (or `pixi shell -e <environment>` to
enter it and drop the `-e` from then on).

### Environments

| Environment        | Select with                              | What it's for                                                         |
| ------------------ | ---------------------------------------- | --------------------------------------------------------------------- |
| `default`          | `pixi run -e default <command>`          | Base ROS 2 workspace: hardware, description, autonomy, input, sensors |
| `simulation`       | `pixi run -e simulation <command>`       | Adds Gazebo (`ros_gz_*`, `gz_ros2_control`) for simulating the robot  |
| `machine-learning` | `pixi run -e machine-learning <command>` | Adds CUDA toolkit + `onnxruntime-cpp` (GPU) for `perseus_vision`      |
| `docs`             | `pixi run -e docs <command>`             | Sphinx documentation build toolchain                                  |
| `format`           | `pixi run -e format <command>`           | `treefmt` + formatters (clang-format, ruff, prettier, etc.)           |

### Commands (once you're in an environment)

Which packages `build`/`build-test`/`test` cover depends on the environment
(see the table above) — `default` skips `perseus_lite_simulation` and
`perseus_vision`, `simulation` includes `perseus_lite_simulation`,
`machine-learning` includes `perseus_vision`.

| Command                 | Environment(s)                              | What it does                                                            |
| ----------------------- | ------------------------------------------- | ----------------------------------------------------------------------- |
| `pixi run build`        | `default`, `simulation`, `machine-learning` | `colcon build` the workspace                                            |
| `pixi run build-test`   | `default`, `simulation`                     | Same build, with `-DBUILD_TESTING=ON`                                   |
| `pixi run test`         | `default`, `simulation`                     | `colcon test` the workspace's test suite                                |
| `pixi run bringup`      | `default`, `simulation`, `machine-learning` | Launch the robot (`perseus_lite.launch.py`) — real hardware             |
| `pixi run bringup-mock` | `default`, `simulation`, `machine-learning` | Same launch, with mock hardware (`use_mock_hardware:=True`)             |
| `pixi run clean`        | `default`, `simulation`, `machine-learning` | Remove `build/`, `install/`, `log/` from the colcon workspace           |
| `pixi run sim`          | `simulation`                                | Build, then launch the full Gazebo simulation (`perseus_sim.launch.py`) |
| `pixi run docs`         | `docs`                                      | Build the Sphinx documentation site                                     |
| `pixi run fmt`          | `format`                                    | Apply formatting across the repo (`treefmt`)                            |
| `pixi run fmt-check`    | `format`                                    | Check formatting without writing changes (CI mode)                      |

See `pixi.toml` for the full environment/dependency definitions and
`CLAUDE.md` for build details, migration notes, and known issues.

## Documentation

Full documentation is located [here](https://roar-qutrc.github.io/)

## CI/CD Status

[![CI](https://github.com/DingoOz/perseus-lite/actions/workflows/all.yaml/badge.svg)](https://github.com/DingoOz/perseus-lite/actions/workflows/all.yaml)
