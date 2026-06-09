# Nix Flake Reference

All predefined commands available via the `perseus-lite` flake.

---

## `nix run` — Launch Applications

Runnable apps that launch ROS 2 nodes, tools, or utility scripts.

```{list-table}
:header-rows: 1
:widths: 25 50 25

* - Command
  - Description
  - Launches
* - `nix run`
  - **Default** — full perseus bringup (v2)
  - `perseus.launch.py`
* - `nix run .#perseus-lite`
  - Lite robot bringup (skid-steer, ST3215 servos)
  - `perseus_lite.launch.py`
* - `nix run .#perseus`
  - Full perseus bringup (v2, alias of default)
  - `perseus.launch.py`
* - `nix run .#generic_controller`
  - Gamepad/keyboard input controller
  - `controller.launch.py`
* - `nix run .#ros2`
  - Drop into the `ros2` CLI with all workspace packages
  - `ros2` binary
* - `nix run .#clean`
  - Wipe all `build/`, `install/`, `log/`, `result/`, `generated/` dirs
  - Shell script
```

```{tip}
Pass extra args after `--`:
`nix run .#perseus-lite -- use_sim_time:=true`
```

---

## `nix develop` — Development Shells

Interactive shells pre-loaded with tools, ROS packages, and environment variables.

```{list-table}
:header-rows: 1
:widths: 30 70

* - Command
  - Environment
* - `nix develop`
  - **Default** — ROS 2 Jazzy workspace with CycloneDDS, Groot2, RViz, Nav2, rosbag2, teleop tools, formatters, Open3D (x86_64)
* - `nix develop .#simulation`
  - Default + Gazebo simulation packages
* - `nix develop .#machineLearning`
  - Default + Gazebo + full CUDA toolkit (nvcc, cuDNN, ONNX Runtime w/ CUDA)
* - `nix develop .#docs`
  - Sphinx documentation build environment (Python/uv)
```

```{note}
All shells set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `ROS_DOMAIN_ID=51` (dev) by default.
Production domain is `42`.
```

---

## `nix build` — Build Packages

Buildable derivations for CI, caching, and deployment.

```{list-table}
:header-rows: 1
:widths: 30 70

* - Command
  - Output
* - `nix build`
  - **Default** ROS workspace (all lite-relevant packages compiled)
* - `nix build .#simulation`
  - Simulation workspace (includes Gazebo deps)
* - `nix build .#machineLearning`
  - ML workspace (CUDA-enabled)
* - `nix build .#docs`
  - Sphinx documentation site (HTML output)
* - `nix build .#pkgs`
  - Passthrough to full nixpkgs overlay (debugging)
* - `nix build .#tools`
  - Utility passthrough (treefmt config, formatters)
* - `nix build .#scripts`
  - All wrapped scripts (clean, machine-setup, cachix helpers, cuda-test)
```

---

## `nix fmt` — Format Everything

```bash
nix fmt
```

Runs [`treefmt`](https://github.com/numtide/treefmt) with the project config — covers C++ (clang-format), Python (ruff), Nix (nixfmt), Markdown, YAML, and more.

---

## Binary Caches

The flake is configured with substituters so most builds are cache hits:

| Cache                     | Key prefix                      |
| ------------------------- | ------------------------------- |
| `perseus-lite.cachix.org` | `perseus-lite.cachix.org-1:...` |
| `roar-qutrc.cachix.org`   | `roar-qutrc.cachix.org-1:...`   |
| `ros.cachix.org`          | `ros.cachix.org-1:...`          |

---

## Quick Copy-Paste

```bash
# Enter dev shell (direnv does this automatically)
nix develop

# Build and run the lite robot
nix run .#perseus-lite

# Launch a gamepad controller
nix run .#generic_controller

# Access ros2 CLI without sourcing anything
nix run .#ros2 -- topic list

# Nuke build artifacts
nix run .#clean

# Build docs locally
nix build .#docs

# Format the entire repo
nix fmt
```
