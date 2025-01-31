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
- `scripts`: Short bash scripts (and Nix files containing scripts) for some commonly run tasks.
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

Applications which use OpenGL (`rviz2`, `gz`, anything which depends on OGRE) will fail to run on non-NixOS systems by default, and likely with obscure and unintelligible error messages. You need to run them using `nixgl COMMAND` instead (assuming you're in the `direnv` environment), which will give it access to the proper OpenGL drivers, and set some required environment variables to keep QT (a GUI library) happy.
It may take a while to build and download drivers and packages the first time you run it, this is fine.

For example: `nixgl rviz2` will run `rviz2`, or `nixgl gz sim` will launch a `gz` simulation window.
