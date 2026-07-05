# About

This folder contains:

- All code running on the rover
- The configuration files for the Pixi-managed dev/build environments
- Machine-specific setup and configuration files
- Tooling helper scripts

Its subdirectories are as follows. Each one contains its own README.md detailing its purpose and contents.

- `machines`: Machine-specific configuration and setup files.
- `native`: Pure C/C++/Python/etc programs which are built and run directly on machines.
- `ros_ws`: The ROS 2 software workspace containing all the actual logic.
- `shared`: Libraries which are shared between the `native` and `ros_ws` workspaces, as well as potentially firmware as well.
- `templates`: Template files for bringing up new projects.
- `scripts`: Short bash scripts for some commonly run tasks.
- `web_ui`: The rover web interface.

# Build System

This project uses [Pixi](https://pixi.sh) (with ROS 2 and its dependencies pulled from the `robostack-jazzy` conda channel) to provide the dev/build toolchain, then invokes `colcon` inside it to actually build the `ros_ws` workspace. See the root `pixi.toml` for the environments available (`default`, `simulation`, `machine-learning`, `docs`, `format`) and `CLAUDE.md` for the quick-start commands.

This gets you:

- Reproducibility: `pixi.lock` pins exact package versions, so a clean `pixi install` reproduces the same environment on any machine.
- Distro independence: RoboStack packages ROS 2 for conda, so the same environment works regardless of your host distro's own package versions.
- No system ROS install required: Pixi environments are self-contained under `.pixi/`, so they don't conflict with (or require) a system-wide ROS installation.

## Overview

Each project is built using standard tools (eg CMake via `ament_cmake`/colcon); Pixi just supplies the toolchain and dependencies that `colcon build` needs.
