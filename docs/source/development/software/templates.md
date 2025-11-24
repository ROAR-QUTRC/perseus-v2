# Templates

There is a set of templates for ROS2 nodes which can be found at `software/templates/ros2`. These include a pair of C++ header and source files with basic functionality, as well as a python launch file. There are also some CMake snippets in `software/templates` that can be used to set up the node properly. When making a ROS2 node, this command should set up most of the structure:

```
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

Then the template files can be placed into the new `src` and `include` folders. Check out the README.md in the `software/templates/cmake_snippets/` directory for which CMake snippets should be included in your node.
