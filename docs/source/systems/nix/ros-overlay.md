# Nix Ros Overlay

This [__overlay__](https://wiki.nixos.org/wiki/Overlays) allows us to have all of the [ROS2 packages](https://index.ros.org/#jazzy) as well as all of the [nixpkgs](https://search.nixos.org/packages) available in our repo.
This means that any library (or program) you need that can be found in either of these can be added to our nix environment and used in development.
Checkout the [nix-ros-overlay repo](https://github.com/lopsided98/nix-ros-overlay) for details on how it works.
The version of ROS2 that we are using is *jazzy*, which is not the newest version, so if you've found a package that needs ROS2 *humble* or later, you might need to find a different package to use.
We have also set our nixpkgs version to follow the nix-ros-overlay's nixpkgs version, so the nixpkgs version will also not be the newest version.

Our nix environment knows what nix packages are needed by our ROS2 packages from the `/software/ros_ws/nix-packages/` folder.
Each of our ROS2 packages has a `.nix` file describing the inputs needed to build the package.
These files are automatically generated, so *please don't edit them manually*.
If you want to update them, make changes to the ROS2 package's `package.xml` file, putting any dependencies in `<depend> </depend>` tags (or the [relevant substitute](https://docs.ros.org/en/jade/api/catkin/html/howto/format2/catkin_library_dependencies.html)).
When your `package.xml` has the required dependencies, run the [`nix-packages.sh`](<project:/systems/software/scripts.md#nix-packages-sh>) script (`/software/scripts/nix-packages.sh`).

:::{note}
You also need to add the dependencies to the CMake file for the package so that the program can build correctly - see <project:/systems/software/ros.md>.
:::