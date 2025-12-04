# Templates

There is a set of templates for ROS2 nodes which can be found at `software/templates/ros2`. These include a pair of C++ header and source files with basic functionality, as well as a python launch file. There are also some CMake snippets in `software/templates` that can be used to set up the node properly.

## Creating the Node

When making a ROS2 node, this command should set up most of the structure:

```
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

Then the template files can be placed into the new `src` and `include` folders. Check out the README.md in the `software/templates/cmake_snippets/` directory for which CMake snippets should be included in your node.

## Building the node

After creating the necessary files, you can build the ROS2 node using colcon. Colcon is used as part of the ROS framework to build the packages and allow them to be run directly within ROS. To build the packages, navigate to the `software/ros_ws/` folder and run:

```
colcon build
```

If you have configured your files correctly you should see an ending output similar to `Summary: X packages finished [~~.~s]`.

## Running the Node

When you have successfully built your package, you can then run it to test the functionality. When testing a package (especially in isolation), you can run it using ROS directly. It is recommended to use a separate command window to see all the outputs, especially if running multiple nodes at once.

To run the package, navigate to the `software/ros_ws/` folder and use the following commands:

```
. install/setup.sh
ros2 run my_package my_node
```

The first line only needs to be run once for every console window. If you get the error `Package 'my_package' not found` make sure you have run the first line inside the `software/ros_ws/` folder.

After running these commands the node within the specified package should start running and outputting the results to console. For more details on this command see [the ROS2 Jazzy Docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html).
