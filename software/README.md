# About

This folder contains:

- All code running on the rover
- The configuration files for the Nix build system
- Machine-specific setup and configuration files
- Tooling helper scripts

Its subdirectories are as follows. Each one contains its own README.md detailing its purpose and contents.

- `machines`: Machine-specific configuration and setup files.
- `native`: Pure C/C++/Python/etc programs which are built and run directly on machines.
- `ros_ws`: The ROS 2 software workspace containing all the actual logic.
- `shared`: Libraries which are shared between the `native` and `ros_ws` workspaces, as well as potentially firmware as well.
- `templates`: Template files for bringing up new projects.
- `tooling`: Short bash scripts containing commonly run tasks.
- `web_ui`: The rover web interface.

# Build System

This project is built with [Nix](https://nixos.org) rather than `colcon` directly, as this allows:

- Reproducibility: If it builds on your machine, it builds on _any_ machine.
- Distro independence: Although ROS 2 is targeted at specific LTS versions of Ubuntu, with Nix it can be built and run on any Linux machine with Nix installed.
- Binary caching: If the built files have been uploaded, you can download them instead of having to build the project locally.
- VSCode integration: Since Nix sets up environment variables, you can now edit ROS 2 code in VSCode without errors and with autocomplete!
- No submodules: Nix can build projects from Git repositories, so you never have to clone it yourself. This means that `git submodule`s can be entirely removed.

## Overview

Each project is built using standard tools (eg CMake), Nix just wraps that in a standard

## Broken OpenGL applications

Applications which use OpenGL (`rviz2`, `gazebo`) will fail to run on non-NixOS systems by default. You need to run them using the command `NIXPKGS_ALLOW_UNFREE=1 QT_QPA_PLATFORM=xcb QT_SCREEN_SCALE_FACTORS=1 nix run --impure github:nix-community/nixGL -- COMMAND` instead, which will allow it to see the proper OpenGL drivers.

- `NIXPKGS_ALLOW_UNFREE=1` allows the use of unfree software packages such as NVIDIA drivers.
- `QT_QPA_PLATFORM=xcb` fixes QT because it likes to have strange issues no matter what.
- `QT_SCREEN_SCALE_FACTORS=1` fixes QT on HiDPI displays generally, otherwise it has some crazy flickering.
- `nix run` runs the default package in the input provided to it, that being `github:nix-community/nixGL`.
- `--impure` tells `nix run` to use environment variables (`QT_QPA_PLATFORM` and `NIXPKS_ALLOW_UNFREE`).
- `--` passes further arguments to the package evaluated by `nix run`.
- `COMMAND` tells the `nixGL` tool what to run. This is what you would normally run on the shell - eg `rviz2` or `gazebo` just like normal.

For example: `NIXPKGS_ALLOW_UNFREE=1 QT_QPA_PLATFORM=xcb QT_SCREEN_SCALE_FACTORS=1 nix run --impure github:nix-community/nixGL -- rviz2` will run `rviz2`.

This command _will_ take a while to evaluate the first time while it downloads the relevant drivers and packages, but that will all be cached the next time around.
