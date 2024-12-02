# Software

The software on the rover is split into the core software (mainly ROS2 code), the web control UI, and firmware.
Since the web UI runs on a completely different stack to the rest of the rover code, it just made more sense to separate it out and treat it separately.
Firmware is compiled completely separately to the rest of the stack, so also needs to be split out (although it does share some libraries with the core software).
:::{tip}
If you want to read through project usage, that'll be documented either in the source code (particularly for libraries) and as such be visible in the [generated documentation](project:/generated/index.rst), or in a README file in the project's directory.
:::

## CMake Build Infrastructure

This project's C++ code (excluding the firmware) is actually targeted at being built in two different ways - first by using Nix, and second by using ordinary `cmake` and `make` commands, both of which have slightly different environments.

The first major difference between Nix and standard builds is handling dependencies.
When the project is built using Nix, all of the dependencies that a target specifies are made available to it like they've been installed and the project can build with no particular extra steps.
However, when built through CMake normally, the individual projects need to manually add the subdirectories of their dependencies.
They can detect whether the packages is present (installed) using the `find_package` command, and only attempt to manually include their dependencies if they aren't already available.

The other major change is related to the [build type](https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html).
When you run CMake manually, there is by default no `CMAKE_BUILD_TYPE` specified.
In this instance, the projects have some additional config in their `CMakeLists.txt` files to default to the `Debug` release type.
Nix builds, by contrast, _specify_ the `Release` build type.
The reason this is important is because these projects add the `-Werror` to the GCC build flags for `Release` builds, thus enforcing the [no warnings](project:/standards/software.md#warnings-are-not-acceptable) section of the software standards.

If you're curious as to what all this looks like in practice, check out the `software/templates/` directory.

## Core

This is mostly ROS code, and is located in `software/ros_ws/src/`.
Since it's a ROS2 project, it's comprised of code in two languages - C++ and Python.
Although, as detailed in the [standards](project:/standards/software.md), we try to keep all the software to C++, there are some cases for which Python just makes more sense (such as input handling - see the `input_devices` package).
The C++ code is all built using [CMake](https://cmake.org/) since that's what ROS2 uses by default, and extending that to non-ROS code allows easy interoperability, as you'll see shortly.

Internally, the code is split into several sections:

- [`software/native/`](project:#dir_native): Programs which run natively, independent of ROS
- [`software/ros_ws/`](project:#dir_ros_ws): Workspace containing ROS2 code
- [`software/shared/`](project:#dir_shared): Shared libraries between native and ROS2 code, and sometimes firmware too

### Native Programs

Currently, there's nothing of note in this category.

### ROS2 Software

This is the digital heart of the rover, and contains pretty much everything which runs its day-to-day operations.
By convention for ROS2 projects, all the actual code in this directory is located under the `src/` subdirectory - everything else in `ros_ws/` is build infrastructure.
The most important packages are are detailed below - if you want more information, there should be `README` files in each package's source directory.

#### `perseus_bringup`

This is a "meta-package" which depends on the other packages and contains launch files for the main tasks needed to bring up the rover.

#### `hardware_interfaces`

This contains implementations of all hardware-specific tasks, and should be one of the only places in the ROS code which interacts directly with the real world.
If hardware-specific code is distributed throughout the codebase, it makes mocking for tests and simulation much more difficult than it needs to be.
There are two types of output inside this package: [Hardware Components](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html) for `ros2_control` and follow its spec, and [nodes](inv:ros#Concepts/Basic/About-Nodes) which interact with other software using either [topics](inv:ros#Concepts/Basic/About-Topics), [actions](inv:ros#Concepts/Basic/About-Actions), or [services](inv:ros#Concepts/Basic/About-Services).

The reason for using `ros2_control` instead of standard nodes and topics is very simple: Speed.
`ros2_control`, rather than launching ROS2 nodes, calls functions directly.
Whilst this can make implementation harder to understand than something similar based on topics, the advantages vastly outweigh the slight additional complexity, especially given its excellent documentation, and the wealth of resources dedicated to explaining it.

#### `input_devices`

This package is the other place which should contain software which interacts directly with the real world, and contains nodes which handle reading input from various devices in the real world.
The nodes in this package then publish data which `ros2_control` reads in, processes with a controller, and feeds to the relevant hardware interface(s).

#### `autonomy`

This package contains the core mapping and autonomous navigation functionality for the rover, as well as the mapping functionality and configuration.
It also implements the fail-over functionality which handles autonomous recovery on disconnection or network failure.

### Shared Libraries

#### Hi-CAN

Abbreviated from "hierarchical CAN", Hi-CAN is the library implementing the standards laid out [here](project:/architecture/can-bus.md), and is shared across ROS and native code, as well as firmware.
The main library contains the code defining the main interfaces with which code will interact with the library, as well as all of the devices on the bus and their parameters.
:::{warning}
Since this particular library is shared between both the ROS code _and_ the firmware, it needs to be written in pure C++ (no external dependencies).
:::
However, this on its own is not particularly useful - which is where implementations come in.
The `hi-can-raw` library implements {class}`hi_can::FilteredCanInterface` using the Linux SocketCAN [`RAW_CAN`](https://docs.kernel.org/networking/can.html#raw-protocol-sockets-with-can-filters-sock-raw) interface, and is what most code uses to interface with the CAN bus.
The `hi-can-net` library is currently unused, but may become an implementation of {class}`hi_can::CanInterface` which forwards all traffic over a network connection.

## Web UI

The other project is the rover's web control UI - since it's a completely different tech stack, it ends up being almost entirely independent of the rover core software.

### Frontend

### Backend

## Firmware

## Nix outputs

This section documents the outputs made available from the project `flake.nix`.
See <project:/home/nix-basics.md> for more information.

### Packages

Intended for use with `nix build`.

#### `default`

A package containing all core software.

#### `simulation`

Same as the `default` package, but contains everything needed to run simulation as well.

### Executables

Intended for use with `nix run`.

#### `default`

Currently just for testing. Will eventually run the main bringup launch file.

#### `ros2`

ROS2 core executable from the `default` package.

### Development shells

Intended for use with `nix develop`.

#### `default`

Workspace containing everything needed to build and run the core software.

#### `simulation`

Same as the `default` dev shell, but extended with the packages needed to run simulation.
