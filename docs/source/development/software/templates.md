# Templates

:::{warning}
Before starting to develop any code, make sure to run `nix develop` to enter the development environment.
:::

When first starting to write code in Ros2, it can be difficult to know where to start. To provide guidance there is a template ROS2 package which can be found at `software/templates/ros2`. This package includes a pair of C++ header and source files with basic functionality, as well as a python launch file. There are also some CMake snippets in `software/templates` that can be used to set up additional functionality within the code.

:::{note}
A Package in Ros is a collection of relevant functionalities (nodes) that achieves a specific purpose. For example, the perseus_input library provides an interface to use local device inputs to control perseus. For more information on packages and nodes alongside Ros development structure, see the [Ros2 Jazzy Docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html).
:::

## Creating the Node

It is best to use the template file structure, as it is best suited to the software standards and allows more flexibitlity within the code structure. However, when making a ROS2 node you can also use the below command to set up most of the structure:

```
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

This will create a file structure slightly different from the template, however, should be relatively simple to fix. After running the command the template files can be placed into the new `src` and `include` folders. Check out the README.md in the `software/templates/cmake_snippets/` directory for which CMake snippets should be included in your node. If you just want to get something basic up and running, see **Example Template** below.

## Building the node

:::{note}
Make sure you are in the root directory before completing the below steps
:::

Before building the node, you need to enter the nix develop shell, as per [the core software sepcifications](https://roar-qutrc.github.io/systems/software/core.html). 

```
nix develop
```

After entering the develop shell, stage your commits to git (locally) before attempting to build with nix.if you don't stage your changes nix will not be able to see the new Ros2 package and the build will fail. After staging your commits run the below command to configure any shared libraries or dependencies for the package (specified in `package.xml`).

```
./software/scripts/nix-package.sh
```

After completing the above steps, you can build the ROS2 node using colcon. Colcon is used as part of the ROS framework to build the packages and allow them to be run directly within ROS. To build the packages, navigate to the `software/ros_ws/` folder and run the following commands:

```
colcon build
```

If you have configured your files correctly you should see an ending output similar to `Summary: X packages finished [XX.Xs]`.

## Running the Node

When you have successfully built your package, you can then run it locally to test the functionality. When testing a package (especially in isolation), you can run it using ROS directly. It is recommended to use a separate command window to see all the outputs, especially if running multiple nodes at once.

To run the package, navigate to the `software/ros_ws/` folder and use the following commands:

```
ros2 run my_package my_node
```

The first line only needs to be run once for every console window. If you get the error `Package 'my_package' not found` make sure you have run the first line inside the `software/ros_ws/` folder.

After running these commands the node within the specified package should start running and outputting the results to console. For more details on this command see [the ROS2 Jazzy Docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html).

## Example Template

To test the above methods, you can use the template package located in `software/templates/ros2`. Start by creating a new branch to test this code (or you alternatively implement in your current branch).

```
git branch feat/new_branch_name
git checkout feat/new_branch_name
```

Create a new folder in the directory `/software/ros_ws/src` called `template_package`. This will be the name of the ros2 package. Copy all the files located in `/software/templates/ros2` into the folder you just created. If you named your folder `template_package` you should be able to run and test the node by following the instructions above.

### Renaming the package

To rename the package you must make several changes.

- First rename the folder you just created in `/software/ros_ws/src` to the new package name
- Next navigate to this renamed folder
- Open `CMakeLists.txt`
  - If using the provided template, change line 5 to the new name of the package. e.g. (`template_package` -> `my_new_package`)
  - If using a custom file (from the CMake snippets) the line to change should be in the first few lines of your CMakeLists.txt file. It is the project name value in the `project()` line from the `01-project_setup.cmake` file.
- Lastly, open `package.xml` and update the name field from `template_package` to `my_new_package`. Also update the description and email to the relevant details if you have not done so already.

### Renaming the node

To rename the node requires a similar process to the package. In this case, the instructions are for a node called `my_node_name`. If using a different name, replace `my_node_name` with your own value.

- Navigate to your new package
- Rename the folder and file in the `src` directory (`template_node/template_node.cpp` -> `my_node_name/my_node_name.cpp`)
- Then rename the file and folder in the `include` directory (`template_node/template_node.hpp` -> `my_node_name/my_node_name.hpp`)
- Open `CMakeLists.txt`
  - If using the provided template, change line 8 from `set(NODE_NAMES template_node)` to `set(NODE_NAMES my_node_name)`
  - If using a custom file (from the CMake snippets) the line to change should be in the first few lines of your CMakeLists.txt file. It is in the `set(NODE_NAMES template_name_here)` line from the `01-project_setup.cmake` file.
- After changing the file names, the relative imports within `./src/template_node/template_node.cpp` need to be updated. Change the first line from `#include "template_node/template_node.hpp"` to `#include "my_node_name/my_node_name.hpp"`

:::{tip}
For more in-depth information on customising the template see the [Ros2 Package Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html). It is recommended to setup and run this template first before attempting to follow the tutorial.
:::
