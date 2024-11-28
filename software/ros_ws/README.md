# About

This directory is the root workspace for rover ROS code.

# Creating new packages

## C++

`cd` to the `src` folder and run `ros2 pkg create --build-type ament_cmake <package_name>`.

In the newly generated folder, open `CMakeLists.txt` and apply the following changes.

1. On line 5, add `-Werror` to the list of compile options - it should now look like this: `add_compile_options(-Wall -Wextra -Wpedantic -Werror)`.
2.
