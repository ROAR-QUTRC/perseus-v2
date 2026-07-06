# Mission Control TUI

`perseus_lite_tui` is a keyboard-driven terminal UI for launching, monitoring,
and stopping the perseus-lite software stack from a single screen. It is SSH- and
headless-friendly (Python standard-library `curses`, no external UI
dependencies) and works with or without ROS sourced — the topic monitor simply
disables itself if `rclpy` is unavailable.

## Running it

```console
pixi run tui
```

That builds nothing; it sources the workspace overlay and runs the TUI. If the
workspace has not been built yet, launch entries report "Workspace not built —
run Tasks → Build first". You can also run it directly inside a `pixi shell`:

```console
source software/ros_ws/install/setup.bash
ros2 run perseus_lite_tui perseus_tui
```

## Tabs

| Tab         | What it does                                                                                                                                                                                                     |
| ----------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Launch**  | Pick a launch profile (bringup, SLAM/Nav2, teleop, simulation) and start it. Profiles with options — e.g. `use_mock_hardware` on bringup — are editable inline.                                                  |
| **Tasks**   | Run one-shot developer commands (`build`, `build-test`, `test`, `clean`, sim build, `fmt`) with live output.                                                                                                     |
| **Monitor** | Live per-topic publish rate and freshness, refreshed continuously. Curated topics (`/cmd_vel`, `/odom`, `/joint_states`, `/scan`, `/tf`, `/map`, `/diagnostics`) sort first; green means fresh, red means stale. |
| **Logs**    | Scrollback for every running or finished job, with stop and clear controls.                                                                                                                                      |

## Keys

| Key            | Action                                                 |
| -------------- | ------------------------------------------------------ |
| `1`–`4`, `Tab` | Switch tab                                             |
| `↑` / `↓`      | Move selection (Launch/Tasks) or scroll (Monitor/Logs) |
| `←` / `→`      | Change the focused option (Launch)                     |
| `l` / `Enter`  | Launch the selected profile / run the selected task    |
| `r`            | Reset the selected profile's options to defaults       |
| `[` / `]`      | Switch job (Logs)                                      |
| `k`            | Stop the selected job (Logs)                           |
| `x`            | Clear finished jobs (Logs)                             |
| `q` / `Esc`    | Quit (all launched jobs are stopped on exit)           |

Option selections persist between runs under `$XDG_CONFIG_HOME/perseus_lite_tui/`.

## How launches are wrapped

Every profile and task is run through `pixi run -e <env>`, so the correct Pixi
environment activates first (including the path-sanitize hook). This is why the
**simulation** and **mission** profiles work even when the TUI itself is running
in the `default` environment: they are routed through `pixi run -e simulation`,
which builds and runs against the Gazebo environment. Those profiles are hidden
on `aarch64`, where the simulation environment has no packages.

Each launch runs as its own process group. Stopping a job sends `SIGINT` to the
whole group and escalates to `SIGTERM` then `SIGKILL` if needed, so `ros2 launch`
gets a chance to shut its nodes down cleanly.

:::{note}
The `teleop_diagnostics` TUI is listed but disabled in the Launch tab: it is
itself a full-screen curses program and cannot share the terminal with a managed
job. Run it in a separate terminal instead.
:::
