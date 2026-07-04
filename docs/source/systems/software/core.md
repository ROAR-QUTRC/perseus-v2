# Core

This is mostly ROS code, and is located in `software/ros_ws/src/`.
Since it's a ROS2 project, it's comprised of code in two languages - C++ and Python.
Although, as detailed in the [standards](project:/standards/software/general.md), we try to keep all the software to C++, there are some cases for which Python just makes more sense (such as input handling - see the `input_devices` package).
The C++ code is all built using [CMake](https://cmake.org/) since that's what ROS2 uses by default, and extending that to non-ROS code allows easy interoperability, as you'll see shortly.

## System architecture at a glance

The diagram below shows how data moves from operator input, through the ROS 2 graph, and out to the four ST3215 wheel servos.
SLAM, Nav2 and vision sit alongside teleop as additional command sources, all multiplexed by `twist_mux`.

```{graphviz}
:caption: Perseus-Lite ROS 2 control & autonomy graph
:align: center

digraph perseus_lite_arch {
    graph [rankdir=LR, splines=spline, bgcolor="transparent", fontname="Roboto",
           pad=0.25, nodesep=0.35, ranksep=0.55];
    node  [shape=box, style="rounded,filled", fontname="Roboto",
           fontsize=11, margin="0.18,0.10", penwidth=1.1];
    edge  [fontname="Roboto", fontsize=9, color="#7a6cad", penwidth=1.1];

    // ---- Operator-facing inputs ----
    subgraph cluster_input {
        label="Operator input"; style="rounded,dashed";
        color="#9c6cff"; fontcolor="#9c6cff"; fontsize=11;
        gamepad [label="Xbox / gamepad\n(input_devices)", fillcolor="#5e35b1", fontcolor="white"];
        keyboard [label="teleop_twist_keyboard", fillcolor="#5e35b1", fontcolor="white"];
        webui   [label="Web UI\n(rosbridge)", fillcolor="#5e35b1", fontcolor="white"];
    }

    // ---- Autonomy ----
    subgraph cluster_auto {
        label="Autonomy"; style="rounded,dashed";
        color="#00bcd4"; fontcolor="#00bcd4"; fontsize=11;
        slam    [label="slam_toolbox", fillcolor="#00838f", fontcolor="white"];
        nav2    [label="Nav2 stack", fillcolor="#00838f", fontcolor="white"];
        bt      [label="Behaviour trees\n(perseus_bt_nodes)", fillcolor="#00838f", fontcolor="white"];
    }

    // ---- Perception ----
    subgraph cluster_perc {
        label="Perception"; style="rounded,dashed";
        color="#ec407a"; fontcolor="#ec407a"; fontsize=11;
        lidar   [label="RPLidar driver", fillcolor="#ad1457", fontcolor="white"];
        imu     [label="I2C IMU driver", fillcolor="#ad1457", fontcolor="white"];
        camera  [label="Camera + ArUco\n(perseus_vision)", fillcolor="#ad1457", fontcolor="white"];
        ekf     [label="robot_localization\nEKF", fillcolor="#ad1457", fontcolor="white"];
    }

    // ---- Core control plane ----
    subgraph cluster_core {
        label="ros2_control plane"; style="rounded,filled";
        color="#3a1f7a"; fillcolor="#1a1340"; fontcolor="#d6c8ff"; fontsize=11;
        twistmux [label="twist_mux", fillcolor="#311b92", fontcolor="white"];
        diffdrive [label="diff_drive_controller\n(TwistStamped)", fillcolor="#311b92", fontcolor="white"];
        hwif    [label="perseus_lite_hardware\n(ST3215 system interface)", fillcolor="#1a237e", fontcolor="white"];
    }

    // ---- Hardware ----
    subgraph cluster_hw {
        label="Hardware"; style="rounded,dashed";
        color="#ec407a"; fontcolor="#ec407a"; fontsize=11;
        serial  [label="/dev/ttyACM0\nFeetech serial bus", shape=cylinder, fillcolor="#37474f", fontcolor="white"];
        fl [label="FL  id 1", shape=circle, fixedsize=true, width=0.7, fillcolor="#ec407a", fontcolor="white"];
        fr [label="FR  id 2", shape=circle, fixedsize=true, width=0.7, fillcolor="#ec407a", fontcolor="white"];
        rl [label="RL  id 3", shape=circle, fixedsize=true, width=0.7, fillcolor="#ec407a", fontcolor="white"];
        rr [label="RR  id 4", shape=circle, fixedsize=true, width=0.7, fillcolor="#ec407a", fontcolor="white"];
    }

    // ---- Input → twist_mux ----
    gamepad  -> twistmux [label="/joy_vel"];
    keyboard -> twistmux [label="/key_vel"];
    webui    -> twistmux [label="/web_vel"];
    nav2     -> twistmux [label="/nav_vel"];

    // ---- Autonomy flow ----
    lidar -> slam    [label="/scan"];
    slam  -> nav2    [label="/map"];
    bt    -> nav2    [label="goals"];
    ekf   -> nav2    [label="/odometry/filtered"];

    // ---- Perception → EKF ----
    imu      -> ekf       [label="/imu/data"];
    hwif     -> ekf       [label="/odom"];
    camera   -> bt        [label="markers / cubes", style=dashed];

    // ---- Control path ----
    twistmux  -> diffdrive [label="/cmd_vel_out", penwidth=2.2, color="#ec407a"];
    diffdrive -> hwif      [label="wheel cmds", penwidth=2.2, color="#ec407a"];
    hwif      -> serial    [label="SYNC_WRITE\n@ ~50 Hz", penwidth=2.2, color="#ec407a"];
    serial    -> fl;
    serial    -> fr;
    serial    -> rl;
    serial    -> rr;
}
```

## Code layout

Internally, the code is split into several sections:

- {file}`software/native/`: Programs which run natively, independent of ROS
- [`software/ros_ws/`](project:#dir_software_ros_ws): Workspace containing ROS2 code
- [`software/shared/`](project:#dir_software_shared): Shared libraries between native and ROS2 code, and sometimes firmware too

## Native Programs

Currently, there's nothing of note in this category.

## ROS2 Software

This is the digital heart of the rover, and contains pretty much everything which runs its day-to-day operations.
By convention for ROS2 projects, all the actual code in this directory is located under the `src/` subdirectory - everything else in `ros_ws/` is build infrastructure.
The most important packages are are detailed below - if you want more information, there should be `README` files in each package's source directory.

### `perseus_lite`

This is the bringup package and contains ROS2 launch files for the main tasks needed to bring up the lite rover (controllers, robot state publisher, SLAM + Nav2, etc.).

### `perseus_lite_hardware`

This contains the `ros2_control` hardware interface for the four Feetech ST3215 wheel servos communicating over serial, and should be one of the only places in the ROS code which interacts directly with the real world.
If hardware-specific code is distributed throughout the codebase, it makes mocking for tests and simulation much more difficult than it needs to be.
There are two types of output inside this package: [Hardware Components](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html) for `ros2_control` and follow its spec, and [nodes](inv:ros#Concepts/Basic/About-Nodes) which interact with other software using either [topics](inv:ros#Concepts/Basic/About-Topics), [actions](inv:ros#Concepts/Basic/About-Actions), or [services](inv:ros#Concepts/Basic/About-Services).

The reason for using `ros2_control` instead of standard nodes and topics is very simple: Speed.
`ros2_control`, rather than launching ROS2 nodes, calls functions directly.
Whilst this can make implementation harder to understand than something similar based on topics, the advantages vastly outweigh the slight additional complexity, especially given its excellent documentation, and the wealth of resources dedicated to explaining it.

### `input_devices`

This package is the other place which should contain software which interacts directly with the real world, and contains nodes which handle reading input from various devices in the real world.
The nodes in this package then publish data which `ros2_control` reads in, processes with a controller, and feeds to the relevant hardware interface(s).

### `autonomy`

This package contains the core mapping and autonomous navigation functionality for the rover, as well as the mapping functionality and configuration.
It also implements the fail-over functionality which handles autonomous recovery on disconnection or network failure.

:::{tip}
The ROS2 build system `colcon` can fail to rebuild cached outputs after events such as a `git pull` or when a non-ROS dependency changes, which may result in `colcon build` incorrectly failing.
The solution is to clean the workspace (`colcon clean workspace -y` or `./software/scripts/clean.sh`) and then re-run `colcon build` (or `pixi run -e default build`).
To ensure that this doesn't happen at all, run a clean after every git pull or after changing any code outside of `software/ros_ws/src`.
:::

## Shared Libraries

:::{tip}
Shared libraries that are packaged for RoboStack/conda-forge can be made available to your ROS2 package by adding them as a `<depend>` in your package's `package.xml`, then adding the corresponding `ros-jazzy-<pkg>` (or conda-forge) entry to `pixi.toml`'s `[dependencies]` and re-running `pixi install` to re-solve the lock file.
Bespoke libraries under `software/shared/` that aren't packaged for RoboStack (e.g. `simple-networking`, `fd-wrapper`) are instead consumed via a CMake `add_subdirectory` fallback - see `perseus_sensors/CMakeLists.txt` for an example.
:::

### Simple-networking

The simple-networking library provides a modern C++ implementation for handling network socket communications, with a primary focus on client-side operations.

It offers an object-oriented wrapper around traditional POSIX socket operations, supporting both TCP and UDP protocols. The library implements RAII principles through its {class}`networking::Client` class, which manages socket creation, configuration, connection and cleanup while providing exception-based error handling for robust failure management.

The library distinguishes itself through flexible socket configuration using handler callbacks, support for custom bind addresses and a clean abstraction over low-level socket operations. It provides convenient methods for transmitting and receiving both string and binary data, with support for both blocking and non-blocking operations. Error handling is comprehensive, with descriptive error messages that include both the operation context and underlying system error details.
:::{warning}
This library is not used for ROS2 communications, it exists for scenarios such as a creating a ROS2 driver node which needs to interface with a specific sensor via ethernet.
:::
