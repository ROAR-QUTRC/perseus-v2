# Core

This is mostly ROS code, and is located in `software/ros_ws/src/`.
Since it's a ROS2 project, it's comprised of code in two languages - C++ and Python.
Although, as detailed in the [standards](project:/standards/software/general.md), we try to keep all the software to C++, there are some cases for which Python just makes more sense (such as input handling - see the `input_devices` package).
The C++ code is all built using [CMake](https://cmake.org/) since that's what ROS2 uses by default, and extending that to non-ROS code allows easy interoperability, as you'll see shortly.

Internally, the code is split into several sections:

- [`software/native/`](project:#dir_native): Programs which run natively, independent of ROS
- [`software/ros_ws/`](project:#dir_ros_ws): Workspace containing ROS2 code
- [`software/shared/`](project:#dir_shared): Shared libraries between native and ROS2 code, and sometimes firmware too

## Native Programs

Currently, there's nothing of note in this category.

## ROS2 Software

This is the digital heart of the rover, and contains pretty much everything which runs its day-to-day operations.
By convention for ROS2 projects, all the actual code in this directory is located under the `src/` subdirectory - everything else in `ros_ws/` is build infrastructure.
The most important packages are are detailed below - if you want more information, there should be `README` files in each package's source directory.

:::{warning}
When creating a new ROS2 package you must stage the ROS2 package in git (locally) before attempting to build with nix. Failure to add to git will result in nix not being able to see the new ROS2 package and your nix build will fail.
:::

### `perseus`

This is a "meta-package" which depends on the other packages and contains ROS2 launch files for the main tasks needed to bring up the rover.

### `perseus_hardware`

This contains implementations of all hardware-specific interfaces and tasks, and should be one of the only places in the ROS code which interacts directly with the real world.
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
The solution is to clean the workspace (`colcon clean workspace -y` or `nix run .#clean`) and then re-run `colcon build`.
To ensure that this doesn't happen at all, run a clean after every git pull or after changing any code outside of `software/ros_ws/src`.
:::

## Shared Libraries

:::{tip}
Shared libraries can be made available to your ROS2 package and nodes by including them as a dependency in your package's package.xml and then running the script 'nix-package.sh'
:::

### Hi-CAN

Abbreviated from "hierarchical CAN", Hi-CAN is the library implementing the standards laid out [here](project:/architecture/can-bus.md), and is shared across ROS and native code, as well as firmware.
The main library contains the code defining the main interfaces with which code will interact with the library, as well as all of the devices on the bus and their parameters.
:::{warning}
Since this particular library is shared between both the ROS code _and_ the firmware, it needs to be written in pure C++ (no external dependencies).
:::
However, this on its own is not particularly useful - which is where implementations come in.
The `hi-can-raw` library implements {class}`hi_can::FilteredCanInterface` using the Linux SocketCAN [`RAW_CAN`](https://docs.kernel.org/networking/can.html#raw-protocol-sockets-with-can-filters-sock-raw) interface, and is what most code uses to interface with the CAN bus.
The `hi-can-net` library is currently unused, but may become an implementation of {class}`hi_can::CanInterface` which forwards all traffic over a network connection.

### Simple-networking

The simple-networking library provides a modern C++ implementation for handling network socket communications, with a primary focus on client-side operations.

It offers an object-oriented wrapper around traditional POSIX socket operations, supporting both TCP and UDP protocols. The library implements RAII principles through its {class}`networking::Client` class, which manages socket creation, configuration, connection and cleanup while providing exception-based error handling for robust failure management.

The library distinguishes itself through flexible socket configuration using handler callbacks, support for custom bind addresses and a clean abstraction over low-level socket operations. It provides convenient methods for transmitting and receiving both string and binary data, with support for both blocking and non-blocking operations. Error handling is comprehensive, with descriptive error messages that include both the operation context and underlying system error details.
:::{warning}
This library is not used for ROS2 communications, it exists for scenarios such as a creating a ROS2 driver node which needs to interface with a specific sensor via ethernet.
:::
