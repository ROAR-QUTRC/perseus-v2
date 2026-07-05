# CLAUDE.md — perseus-lite

Project-specific guidance. Global rules from `~/.claude/CLAUDE.md` (auto-approved
commands, `ERRORS.md` workflow, `feature/<name>` branch workflow, no AI
attribution in git messages) apply here — this file does not restate them.

## 1. Project identity

`perseus-lite` is a fork of [`ROAR-QUTRC/perseus-v2`](https://github.com/ROAR-QUTRC/perseus-v2)
(`upstream` remote already configured; `origin` =
[`DingoOz/perseus-lite`](https://github.com/DingoOz/perseus-lite)).

The **lite robot** is a smaller, simpler subset of the full perseus-v2 platform:

- 50%-scale, 4-wheel skid-steer chassis with rocker suspension
- **Feetech ST3215 servos over serial** for wheel drive (one per wheel,
  IDs 1–4) — no CAN bus, no VESCs
- **Feetech-servo manipulator arm** (controlled by
  `software/arm-teleop-direct`)
- Designed for sandy environments
- Autonomy / SLAM / Nav2 / vision / Gazebo simulation are all in scope

**Explicitly NOT on the lite robot:**

- CAN bus, VESC motor controllers, mecanum drive
- Excavation bucket, elevator module, processing plant, light tower
- Bespoke perseus-v2 control PCBs (rover-control-board, smol-brain-board)

**Current cleanup state:** All four cleanup phases complete. The fork has
hard-diverged from upstream — perseus-v2-only packages, firmware, hardware
designs, and challenge docs are deleted, not just disabled.

## 2. Repository layout

| Path                                                                      | Contents                                                   |
| ------------------------------------------------------------------------- | ---------------------------------------------------------- |
| `software/ros_ws/src/`                                                    | All ROS 2 packages (Jazzy). See §4 for KEEP/DISABLE.       |
| `software/arm-teleop-direct/`                                             | Standalone serial Feetech arm teleop (C++). Lite-relevant. |
| `software/{daemons,scripts,utilities,web_ui,shared,native,home-manager}/` | General system infra — keep.                               |
| `firmware/`                                                               | ESP32/MCU firmware. Subdirs split lite/v2 — see §4.        |
| `docs/`                                                                   | Sphinx docs site (mostly perseus-v2 — see §4).             |
| `pixi.toml`, `pixi.lock`                                                  | Pixi/RoboStack dev environments + build (replaced Nix).    |
| `.clang-format`, `treefmt.toml`                                           | Formatting config — run before commits.                    |

## 3. Build / run quick reference

```bash
# Dev environment via Pixi/RoboStack (direnv loads automatically; otherwise:)
pixi shell                  # default env; ROS 2 Jazzy auto-sourced
# or run tasks directly without entering a shell:
pixi run -e default build   # colcon build (skips sim + vision; see pixi.toml)
pixi run -e default test    # CI unit-test subset
pixi run -e simulation build # full build incl. perseus_lite_simulation (Gazebo)
pixi run -e machine-learning build # full build incl. perseus_vision (CUDA ONNX)

# Manual colcon build inside `pixi shell`:
cd software/ros_ws
colcon build --symlink-install
source install/setup.bash

# Primary lite bringup
ros2 launch perseus_lite perseus_lite.launch.py

# SLAM + Nav2
ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py

# Just controllers / state publisher
ros2 launch perseus_lite controllers.launch.py
ros2 launch perseus_lite robot_state_publisher.launch.py

# Arm teleop (standalone, not a ROS package)
# Build per software/arm-teleop-direct/README.md
```

`ros2_control` requires stamped twist messages on Jazzy. To debug-publish:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.5}}}'
```

### Build system: Pixi/RoboStack (Nix removed)

The project migrated from Nix to **Pixi**. ROS 2 Jazzy + deps come from the
`robostack-jazzy` conda channel; `pixi.lock` pins exact versions. Four
environments: `default` (the robot; linux-64 **and** linux-aarch64), `simulation`
and `machine-learning` (linux-64 only — Gazebo / CUDA have no aarch64 builds),
and `docs`. Run `nix fmt`'s replacement with `pixi run -e format fmt`.

Migration notes:

- **Open3D** is now a conda-forge package (no custom build); **Groot2** has no
  conda package (install the AppImage manually if needed).
- The upstream third-party SLAM/coverage packages (`lidarslam_ros2`,
  `ndt-omp-ros2`, `opennav-coverage`, `fields2cover`) were **dropped** — nothing
  in the lite workspace referenced them. See `software/ros_ws/conda-recipes/`.
- `perseus_vision` builds only in `machine-learning` (needs CUDA `onnxruntime-cpp`).
  It is **not built in CI** — no CI runner has the machine-learning env's CUDA
  toolchain available (documented gap, not implemented).
- `twist_stamper` (a `perseus_lite_simulation` runtime dep) is not in RoboStack;
  it is vendored in-tree as `software/ros_ws/src/twist_stamper` (original code,
  built as a normal workspace package).
- `software/home-manager/` is **orphaned**: it was Nix machine-provisioning with
  no Pixi equivalent. Preserved (not deleted) but non-functional without a flake;
  extract to its own repo or remove.
- `software/web_ui/`'s Node/gstreamer toolchain has no Pixi equivalent yet
  (documented gap, not implemented) — same category as `home-manager` above.
- **Production `ROS_DOMAIN_ID` split is gone**: the old Nix shells used
  release=42 / dev=51; every Pixi env now hardcodes the dev domain (51, see
  `pixi.toml`'s `[activation.env]`). A production/release override mechanism
  has not been ported (documented gap, not implemented).
- Sphinx docs env has no `drawio`/`graphviz` for figure generation (the old Nix
  docs shell had both); see `[feature.docs.dependencies]` in `pixi.toml`
  (documented gap, not implemented).
- **System-ROS contamination hazard**: if a machine also sources a native
  `/opt/ros/jazzy/setup.bash` (or another colcon workspace) from `~/.bashrc`,
  those paths leak into every Pixi env's `LD_LIBRARY_PATH`/`AMENT_PREFIX_PATH`/
  `CMAKE_PREFIX_PATH`/`PYTHONPATH`/`COLCON_PREFIX_PATH`/`GZ_CONFIG_PATH` and can
  cause ABI-mismatch crashes (e.g. `gz sim` segfaulting on a plugin built for a
  different Gazebo version — see `ERRORS.md`). `software/scripts/pixi-env-sanitize.sh`
  (wired via `pixi.toml`'s `[activation] scripts`) strips known-foreign entries
  on every `pixi shell`/`pixi run`, but it is a defensive backstop, not a fix
  for a contaminated parent shell — prefer guarding the offending `~/.bashrc`
  source line so it's skipped inside Pixi/conda shells.
- **Resolved: `gz sim`/`rviz2` GUI segfault from a shared fontconfig cache.**
  `pixi run -e simulation sim` (and any GUI `gz sim`/`rviz2` invocation) used
  to reliably segfault a few seconds into startup, inside `libfontconfig`
  during Qt's text shaping. Root cause: the system's fontconfig (2.15.0) and
  the Pixi `simulation` env's bundled fontconfig (2.18.1) shared the same
  default `~/.cache/fontconfig` directory, and reading a cache built by one
  version with the other corrupted an `FcCharSet` traversal — see `ERRORS.md`.
  Fixed by pointing `XDG_CACHE_HOME` at `$CONDA_PREFIX/var/cache` (a
  Pixi-env-private cache dir) in the `additional_env` of every GUI-launching
  `ExecuteProcess` (`gazebo.launch.py`, `perseus_sim.launch.py`'s RViz,
  `mapping_using_slam_toolbox.launch.py`'s RViz). Verified with multiple
  clean 30-75s GUI runs under `sg render`, zero crashes.

## 4. What's lite-relevant vs upstream-only

### KEEP — used directly by lite

| Package / dir                                                           | Role                                                                                                                                              |
| ----------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `perseus_lite`                                                          | Lite bringup: launch files, controllers, RViz config                                                                                              |
| `perseus_lite_hardware`                                                 | `ros2_control` hardware interface for ST3215 servos over serial                                                                                   |
| `perseus_lite_description`                                              | Lite URDF (4-wheel skid-steer, rocker, scaled meshes). All meshes now self-contained (Phase 2).                                                   |
| `perseus_sensors`                                                       | IMU + lidar drivers (RPLidar)                                                                                                                     |
| `perseus_interfaces`                                                    | Custom msg/srv definitions (shared)                                                                                                               |
| `input_devices`, `perseus_input`, `perseus_input_config`                | Gamepad/keyboard input + routing                                                                                                                  |
| `teleop_diagnostics`                                                    | TUI debug for teleop (shared)                                                                                                                     |
| `autonomy`, `perseus_autonomy_bridge`, `perseus_bt_nodes`, `pcl_to_lsr` | Nav2 / SLAM (slam_toolbox) / behavior trees / pointcloud→laserscan                                                                                |
| `perseus_vision`                                                        | ONNX detectors (cube, ArUco)                                                                                                                      |
| `perseus_lite_simulation`                                               | Gazebo sim forked from `perseus_simulation`; spawns the lite URDF, vendors `twist_mux` config (Phase 3).                                          |
| `software/arm-teleop-direct`                                            | Serial Feetech arm teleop                                                                                                                         |
| `software/shared`                                                       | Shared C++ libs (fd-wrapper, crc, ptr-wrapper, simple-networking, type-demangle); built in-tree by `perseus_sensors` via CMake `add_subdirectory` |
| `software/{daemons,scripts,utilities,web_ui,home-manager,native}`       | General infra (`home-manager` is orphaned Nix machine-config — see note below)                                                                    |

### REMOVED — deleted, not in tree

Phase 1–4 removals:

- ROS packages: `perseus`, `perseus_hardware`, `perseus_can_if`,
  `perseus_payloads`, `perseus_simulation`, `perseus_description`
- Nix package recipes (`software/ros_ws/nix-packages/`): `perseus.nix`,
  `perseus-hardware.nix`, `perseus-can-if.nix`, `perseus-payloads.nix`,
  `perseus-simulation.nix`, `perseus-description.nix`
- Firmware: `firmware/{excavation-bucket,elevator-module,light-tower,processing-plant}`
- Hardware: `hardware/dc-motor-driver`
- Docs: `docs/source/challenge-breakdowns/{excavation-and-construction,space-resources}.md`

Post-Phase-4 v2-residual cleanup (`feature/v2-residual-cleanup` branch):

- Entire `firmware/` tree — `battery-management-system` and
  `components/{board-support,crc,hi-can,type}` are v2-only (lite carries
  no BMS-over-CAN). Also `.github/workflows/firmware.yaml` +
  `.github/filters-firmware.yaml`.
- hi-can shared C++ libs: `software/shared/{hi-can,hi-can-net,hi-can-raw,hi-can-generator}/`,
  `software/native/examples/hi-can/`, `software/scripts/vcan-setup.sh`.
- Livox MID-360 + fast-lio chain: `packages/livox-sdk2/`,
  `software/ros_ws/third-party-packages/{livox-ros-driver2,fast-lio}/`,
  `software/ros_ws/src/perseus_mapping/`,
  `software/ros_ws/nix-packages/perseus-mapping.nix`,
  `software/ros_ws/src/perseus_sensors/launch/livox.launch.py`.
  (Lite uses RPLidar + slam_toolbox.)
- v2-only docs: `docs/source/hardware/{vescs,can-daisy-chain,rover-control-board,smol-brain-board}.md`,
  `docs/source/systems/can-bus.md`,
  `docs/source/tutorials/hi-can-index.md` + `docs/source/tutorials/hi-can/`.

To inspect any of these, use `git log -- <path>` or check upstream
(`ROAR-QUTRC/perseus-v2`) directly.

## 5. Phased cleanup playbook

All four phases are complete; the sections below are kept as a record of
what was done and why. New work on the lite fork should follow §6 for
upstream cherry-picks rather than re-running these phases.

### Phase 1 — disable (safe, reversible) — DONE

`COLCON_IGNORE` markers are in place on:
`perseus`, `perseus_hardware`, `perseus_can_if`, `perseus_payloads`,
and `perseus_simulation` (cascaded — it depends on `perseus`).

Verified: `colcon list` shows 16 packages, `colcon build` succeeds clean.

Firmware modules (`excavation-bucket`, `elevator-module`, `light-tower`,
`processing-plant`) are **not** wired into `flake.nix` and aren't part
of any default build target — no Phase 1 action was needed. They stay
in-tree until Phase 4 deletion.

### Phase 2 — break the `perseus_description` mesh dependency — DONE

Six meshes (`chassis`, `flange_bearing`, `differential_bar`, `rocker_left`,
`rocker_right`, `gearbox`) copied into
`perseus_lite_description/meshes/`. Three xacro files
(`chassis.urdf.xacro`, `rocker.urdf.xacro`, `motor_wheel.urdf.xacro`)
repointed from `$(find perseus_description)` to
`$(find perseus_lite_description)`. `<depend>perseus_description</depend>`
removed from `perseus_lite_description/package.xml`.
`perseus_description` itself disabled via `COLCON_IGNORE`.

Verified: `xacro src/perseus_lite/urdf/perseus_lite.urdf.xacro` resolves
to a 644-line URDF with 18 mesh references, all under
`perseus_lite_description/`, zero stale `perseus_description` refs.
`colcon build` passes with 15 packages.

### Phase 3 — fork `perseus_simulation` → `perseus_lite_simulation` — DONE

Took **Option A** (full fork). `perseus_simulation/` was copied to
`perseus_lite_simulation/` (the original keeps its `COLCON_IGNORE`):

- `package.xml`: `<name>perseus_lite_simulation</name>`,
  `<exec_depend>perseus</exec_depend>` → `<exec_depend>perseus_lite</exec_depend>`,
  maintainer updated.
- `CMakeLists.txt`: `project(perseus_lite_simulation)`.
- `launch/perseus_sim.launch.py`: `FindPackageShare("perseus")` for
  `robot_state_publisher.launch.py` and `controllers.launch.py` swapped to
  `perseus_lite`.
- `launch/gazebo.launch.py`: Gazebo spawn-entity name `"perseus"` →
  `"perseus_lite"`.
- `launch/twist_mux.launch.py` + `config/twist_mux.yaml`: vendored from the
  v2 `perseus` package; the launch file now self-references
  `perseus_lite_simulation`.
- `tests/gz_bridge_config_test.cpp`: hardcoded path updated by the bulk
  rename.

Verified: `colcon build` passes 16 packages; `ROS2 launch --show-args
perseus_lite_simulation perseus_sim.launch.py` resolves the full launch
graph (rsp + controllers + gazebo + rosbridge + twist_mux + ekf + rviz)
without errors. Actually running Gazebo requires `pixi shell -e simulation`
and a GPU — not validated in this batch.

### Phase 4 — hard-divergence delete — DONE

Removed all v2-only packages, firmware, hardware designs, and Nix
recipes (see §4 "REMOVED in Phase 4" for the full list). Also folded
in two leftovers from earlier phases: `perseus-lite-description.nix`
no longer declares `perseus-description` as an input (Phase 2
follow-up), and `perseus-lite-simulation.nix` was created
(Phase 3 follow-up). GitHub firmware workflow trimmed to BMS-only;
simulation tutorial repointed at `perseus_lite_simulation`.

**Local-machine recommendation (not in repo):** rename the upstream
remote so reflexive `git pull upstream main` doesn't pull deleted
packages back: `git remote rename upstream upstream-archive`. Future
upstream tracking should be `git cherry-pick` of specific commits.

## 6. Working with upstream

The fork has hard-diverged. **Do not `git merge upstream/main`** — it will
re-introduce the v2-only packages that Phase 4 deleted. Instead:

```bash
git fetch upstream
git log --oneline HEAD..upstream/main                  # see what's new upstream
git cherry-pick <sha>                                  # pull individual commits
```

If a desired commit touches deleted paths, expect conflicts and resolve by
keeping the deletions (`git rm` the paths during conflict resolution).

## 7. Conventions specific to this repo

- ROS 2 distribution: **Jazzy**. `diff_drive_controller` requires
  `TwistStamped` messages.
- Package licenses are MIT (see recent `chore: Update Nix packaging` and
  `chore: Changed ROS package.xml TODO licenses to MIT` commits).
- C++ formatting: `.clang-format` at repo root. Run `treefmt` before commit.
- Servo IDs are fixed by hardware: FL=1, FR=2, RL=3, RR=4. Don't renumber
  without coordinating with the firmware/wiring side.

## 8. First-time-on-this-repo checklist

1. Read this file end-to-end.
2. `git fetch upstream && git log --oneline HEAD..upstream/main` — see
   pending upstream drift. Use `git cherry-pick`, not `git merge` (§6).
3. `colcon list` in `software/ros_ws/` — should be 15 packages, all on
   the KEEP table in §4.
4. Check `ERRORS.md` (per global rules) for any prevention rules touching
   files you plan to edit. Create `ERRORS.md` and log new bugs as
   instructed in `~/.claude/CLAUDE.md`.
5. Create a `feature/<name>` branch before changes (per global rules).
