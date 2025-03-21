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

:::{note}
Under the hood, Nix's [`stdenv.mkDerivation`](https://nixos.org/manual/nixpkgs/stable/#sec-using-stdenv) automatically detects and uses CMake for these projects.
More on that here: <project:nix.md>.
:::

### Testing

Individual projects are unit tested using [GoogleTest](https://google.github.io/googletest/) (AKA GTest).
This is the default testing solution supported for ROS2 projects, as documented [here](inv:ros#Tutorials/Intermediate/Testing/Cpp).
It works well enough that we're also using it for unit testing the shared and native software.

% TODO: Investigate using catch2 instead of GTest - maybe better?

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

:::{warning}
When creating a new ROS2 package you must stage the ROS2 package in git (locally) before attempting to build with nix. Failure to add to git will result in nix not being able to see the new ROS2 package and your nix build will fail.
:::

#### `perseus`

This is a "meta-package" which depends on the other packages and contains ROS2 launch files for the main tasks needed to bring up the rover.

#### `perseus_hardware`

This contains implementations of all hardware-specific interfaces and tasks, and should be one of the only places in the ROS code which interacts directly with the real world.
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

:::{tip}
The ROS2 build system `colcon` can fail to rebuild cached outputs after events such as a `git pull` or when a non-ROS dependency changes, which may result in `colcon build` incorrectly failing.
The solution is to clean the workspace (`colcon clean workspace -y` or `nix run .#clean`) and then re-run `colcon build`.
To ensure that this doesn't happen at all, run a clean after every git pull or after changing any code outside of `software/ros_ws/src`.
:::

### Shared Libraries

:::{tip}
Shared libraries can be made available to your ROS2 package and nodes by including them as a dependency in your package's package.xml and then running the script 'nix-package.sh'
:::

#### Hi-CAN

Abbreviated from "hierarchical CAN", Hi-CAN is the library implementing the standards laid out [here](project:/architecture/can-bus.md), and is shared across ROS and native code, as well as firmware.
The main library contains the code defining the main interfaces with which code will interact with the library, as well as all of the devices on the bus and their parameters.
:::{warning}
Since this particular library is shared between both the ROS code _and_ the firmware, it needs to be written in pure C++ (no external dependencies).
:::
However, this on its own is not particularly useful - which is where implementations come in.
The `hi-can-raw` library implements {class}`hi_can::FilteredCanInterface` using the Linux SocketCAN [`RAW_CAN`](https://docs.kernel.org/networking/can.html#raw-protocol-sockets-with-can-filters-sock-raw) interface, and is what most code uses to interface with the CAN bus.
The `hi-can-net` library is currently unused, but may become an implementation of {class}`hi_can::CanInterface` which forwards all traffic over a network connection.

#### Simple-networking

The simple-networking library provides a modern C++ implementation for handling network socket communications, with a primary focus on client-side operations.

It offers an object-oriented wrapper around traditional POSIX socket operations, supporting both TCP and UDP protocols. The library implements RAII principles through its {class}`networking::Client` class, which manages socket creation, configuration, connection and cleanup while providing exception-based error handling for robust failure management.

The library distinguishes itself through flexible socket configuration using handler callbacks, support for custom bind addresses and a clean abstraction over low-level socket operations. It provides convenient methods for transmitting and receiving both string and binary data, with support for both blocking and non-blocking operations. Error handling is comprehensive, with descriptive error messages that include both the operation context and underlying system error details.
:::{warning}
This library is not used for ROS2 communications, it exists for scenarios such as a creating a ROS2 driver node which needs to interface with a specific sensor via ethernet.
:::

## Perseus-UI

The Perseus-UI is a web UI that is almost entirely independent of the rover core software. The UI is a SvelteKit + Vite app with an injected Node.js server. There are two main components to Perseus-UI:

- Widgets -> Typically a single svelte component that provide an interface for a specific functionality on the rover.
- Layouts -> A group of widgets that are regularly used together.

### Running Perseus-UI

The Rover UI uses SvelteKit meaning that it does not need a server to be run and can be started by running these commands in the root directory:

```shell
yarn # This installs dependencies
yarn build # Build the server
yarn start # Runs the built server on port 3000
```

**Note:** Any hydration related errors can be ignored

:::{note}
If you don't have node/yarn I'd recommend these steps:

Install node version manager with this script (you will need to restart your shell once its installed):

```
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
```

Install the latest lts version of node:

```
nvm install --lts
```

If you have other versions of node already installed check the [nvm docs](https://github.com/nvm-sh/nvm) on how to setup environments or change the active node version.

Finally, install yarn with:

```
npm install --global yarn
```

:::

### Developing for the UI

#### Client

The UI library used is [shadcn-svelte](https://next.shadcn-svelte.com), with [roslibjs](https://robotwebtools.github.io/roslibjs/index.html) used for communication with the ROS stack.

#### Server

SvelteKit provides a number of methods of assisting with server communication, however I would recommend using the node server and WebSockets to prevent weird bugs from components re-rendering. Since Svelte doesn't provide a native interface for creating custom node servers an injected server is used. For development and testing use write your node server code in the vite plugin inside `vite.config.ts` and copy this code into `web-ui/src/server/server.js` for production (alternatively create a new file that exports a function and call it in both then develop from that file).

#### Widgets

To begin developing a widget run the command: `./create-widget.sh <file-name>`. The `file-name` argument is just the name of the file that contains the widget and **NOT** the widget name. This should generate a new file `/src/lib/widgets<file-name>.svelte` with this template contents:

```svelte
<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'New Widget';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	// Widget logic goes here
</script>

<p>New component</p>
```

- The first script tag with the `module` property is a server only module. This is used here as it is only run once when the component is first loaded and it also allows for exports that are used to expose some properties. You likely will not need to write your own code here.

  - **name -** This string is the unique name of the widget that will be displayed at the top of the widget and is used to ensure duplicates of widgets are not loaded.
  - **settings -** This is the object that structures the settings/state panel of each widget, this will automatically update across all connected devices when saved and will persist between sessions. Here is an example settings object and how to access the value of each setting:

    ```ts
    export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
      groups: {
        general: {
          textSetting: {
            type: "text", // options are: text, number, switch, select, button
            value: "Some text", // This is a default value
          },
          hereIsAButton: {
            type: "button",
            action: () => {
              // action is executed on button press
              console.log("Button clicked");
              return "Success"; // string return value is printed in a toast
            },
          },
        },
        Advanced: {
          booleanSwitch: {
            type: "switch",
            description: "This is a switch", // fields with descriptions get a question mark next to their label
          },
          selectOptions: {
            type: "select",
            options: [
              { value: "1", label: "Option 1" },
              { value: "2", label: "Option 2" },
              { value: "3", label: "Option 3" },
            ],
          },
        },
      },
    });

    console.log(settings.groups.Advanced.booleanSwitch.value); // logs the state of the switch
    ```

- The second script tag is where your code should go as it is run client side when the component is rendered. The settings can be accessed here without an import and to listen for changes wrap the value you want to check in either a `$derived` or `$effect` rune. Furthermore, use `toast('message')` to send a toast from your widget.
- The remainder of the file is where mark down goes. Tailwind classes can be used on widgets for styling or a `<style></style>` block, although the former is strongly preferred if possible.

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

#### `pkgs`

This is the entire `nixpkgs` output, as well as all project package outputs (through overlays).
Having it available can make certain debugging easier.

Specific useful packages/package sets are documented below.

#### `pkgs.rosPackages.${rosDistro}`

All ROS packages for a specific distro.
Packages which contain spaces or underscores in their name have those replaced with dashes.
The packages for the project's target distro will also be modified to add all the ROS2 packages in this workspace - for example, if targeting ROS2 Jazzy, the autonomy package would be accessible under `pkgs.rosPackages.jazzy.autonomy`.

:::{example}
To use `ros2_control` from ROS Rolling Ridley: `pkgs.rosPackages.rolling.ros2-control`.
:::
:::{warning}
If you use these packages with `nix shell`, be aware that packages which apply modifications to the ROS2 CLI may not add the verb correctly.
:::

#### `pkgs.ros`

The entire ROS package set of the distro we're using.
Includes the packages in development.
This is the recommended way to access packages in development if you want to pull specific ones out for `nix build` or `nix shell`.

:::{example}
To use the workspace `autonomy` package: `pkgs.ros.autonomy`.
:::

#### `tools.treefmt-write-config`

Intended for use with `nix run`.
Writes the configuration generated by [`treefmt-nix`](https://github.com/numtide/treefmt-nix) to `treefmt.toml` in the repo root.

#### `tools.treefmt-build`

The `treefmtEval.config.build` set generated by [`treefmt-nix`](https://github.com/numtide/treefmt-nix).

#### `scripts`

Scripts intended for use with `nix run`.
Also available at `pkgs.scripts` (due to being applied as an overlay).

### Executables

Intended for use with `nix run`.

#### `default`

Currently just for testing. Will eventually run the main bringup launch file.

#### `ros2`

ROS2 core executable from the `default` package.

#### `clean`

Convenience alias of [`scripts.clean`](#scripts-clean)

(scripts-clean)=

#### `scripts.clean`

Runs `software/scripts/clean.sh` to remove all generated/build directories from the workspace.

#### `scripts.cachix`

Scripts to push build targets to Cachix.
Current sub-attributes available:

- `build`: Main build
- `shell`: Dev shell environment
- `all`: Both `build` and `shell`, as well packages for simulation
- `docs-shell`: Shell environment for building the documentation

:::{note}
This requires a Cachix auth token to succeed.
Contact James N if you need one.
:::

### Development shells

Intended for use with `nix develop`.

#### `default`

Workspace containing everything needed to build and run the core software.

#### `simulation`

Same as the `default` dev shell, but extended with the packages needed to run simulation.
