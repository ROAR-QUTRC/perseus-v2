# CMake Build Infrastructure

This project's C++ code (excluding the firmware) is actually targeted at being built in two different ways - first by using Nix, and second by using ordinary `cmake` and `make` commands, both of which have slightly different environments.

The first major difference between Nix and standard builds is handling dependencies.
When the project is built using Nix, all of the dependencies that a target specifies are made available to it like they've been installed and the project can build with no particular extra steps.
However, when built through CMake normally, the individual projects need to manually add the subdirectories of their dependencies.
They can detect whether the packages is present (installed) using the `find_package` command, and only attempt to manually include their dependencies if they aren't already available.

The other major change is related to the [build type](https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html).
When you run CMake manually, there is by default no `CMAKE_BUILD_TYPE` specified.
In this instance, the projects have some additional config in their `CMakeLists.txt` files to default to the `Debug` release type.
Nix builds, by contrast, _specify_ the `Release` build type.
The reason this is important is because these projects add the `-Werror` to the GCC build flags for `Release` builds, thus enforcing the [no warnings](project:/standards/software/general.md#warnings-are-not-acceptable) section of the software standards.

If you're curious as to what all this looks like in practice, check out the `software/templates/` directory.

:::{note}
Under the hood, Nix's [`stdenv.mkDerivation`](https://nixos.org/manual/nixpkgs/stable/#sec-using-stdenv) automatically detects and uses CMake for these projects.
More on that here: <project:nix.md>.
:::

## Testing

Individual projects are unit tested using [GoogleTest](https://google.github.io/googletest/) (AKA GTest).
This is the default testing solution supported for ROS2 projects, as documented [here](inv:ros#Tutorials/Intermediate/Testing/Cpp).
It works well enough that we're also using it for unit testing the shared and native software.

% TODO: Investigate using catch2 instead of GTest - maybe better?
