# perseus-v2

The main codebase for perseus starting 2024

## Pixi commands

Dev environment and builds are managed by [Pixi](https://pixi.sh). Run
`pixi install` once, then use `pixi run -e <environment> <command>` — always
pass `-e`/`--environment` explicitly, since several commands (`build`,
`build-test`, `test`) exist in more than one environment with different
behavior.

| Command                               | Environment        | What it does                                                                                        |
| ------------------------------------- | ------------------ | --------------------------------------------------------------------------------------------------- |
| `pixi run -e default build`           | `default`          | `colcon build` the core workspace (skips `perseus_lite_simulation`, `perseus_vision`)               |
| `pixi run -e default build-test`      | `default`          | Same build, with `-DBUILD_TESTING=ON`                                                               |
| `pixi run -e default test`            | `default`          | `colcon test` the CI unit-test subset (`perseus_lite_hardware`, `perseus_lite`)                     |
| `pixi run -e default bringup`         | `default`          | Launch the robot (`perseus_lite.launch.py`) — real hardware                                         |
| `pixi run -e default bringup-mock`    | `default`          | Same launch, with mock hardware (`use_mock_hardware:=True`)                                         |
| `pixi run -e default clean`           | `default`          | Remove `build/`, `install/`, `log/` from the colcon workspace                                       |
| `pixi run -e simulation build`        | `simulation`       | `colcon build` including `perseus_lite_simulation` (Gazebo; skips `perseus_vision`)                 |
| `pixi run -e simulation build-test`   | `simulation`       | Same build, with `-DBUILD_TESTING=ON`                                                               |
| `pixi run -e simulation test`         | `simulation`       | `colcon test` including `perseus_lite_simulation`                                                   |
| `pixi run -e simulation sim`          | `simulation`       | Build, then launch the full Gazebo simulation (`perseus_sim.launch.py`)                             |
| `pixi run -e simulation bringup`      | `simulation`       | Same as the `default` env's `bringup`                                                               |
| `pixi run -e simulation bringup-mock` | `simulation`       | Same as the `default` env's `bringup-mock`                                                          |
| `pixi run -e simulation clean`        | `simulation`       | Same as the `default` env's `clean`                                                                 |
| `pixi run -e machine-learning build`  | `machine-learning` | `colcon build` including `perseus_vision` (CUDA/`onnxruntime-cpp`; skips `perseus_lite_simulation`) |
| `pixi run -e docs docs`               | `docs`             | Build the Sphinx documentation site                                                                 |
| `pixi run -e format fmt`              | `format`           | Apply formatting across the repo (`treefmt`)                                                        |
| `pixi run -e format fmt-check`        | `format`           | Check formatting without writing changes (CI mode)                                                  |

See `pixi.toml` for the full environment/dependency definitions and
`CLAUDE.md` for build details, migration notes, and known issues.

## Documentation

Full documentation is located [here](https://roar-qutrc.github.io/)

## CI/CD Status

[![CI](https://github.com/DingoOz/perseus-lite/actions/workflows/all.yaml/badge.svg)](https://github.com/DingoOz/perseus-lite/actions/workflows/all.yaml)
