# CMake Snippets

This directory contains the snippets needed to build up a full `CMakeLists.txt` file for a project, be it a ROS2 package, a library, or a standalone executable.

Select the snippets you need, and insert them into the project's root `CMakeLists.txt` file in order of the number prepended to the snippet's filename.
If there are two snippets with the same number but one is prefixed with `ros_`, you should prefer to use that one when setting up a ROS project.
Additionally, if it does have the `ros_` prefix, it probably relies on the ament build system and should **not** be used outside of ROS projects.

When using these snippets, read them carefully - many of them require modification depending on the specific project.
These should be quite visible, but it's worth keeping in mind.
The most important items to change are:

- Project name and version
- Project dependencies
- If the project has multiple targets, replace the `${PROJECT_NAME}` string for the `compile_*`, `link_*`, and `install_*` snippets with the names of the individual targets

## Library Setup

Libraries also require copying the `config.cmake.in` file to the library project root directory (the same place as `CMakeLists.txt`).

### Setup

Create a `CMakeLists.txt` file and insert the following snippets in order, editing them as needed:

- `project_setup`: Edit the project name + version.
- (OPTIONAL) `include_dependencies`: Edit the dependency lists.
- (OPTIONAL, non-ROS only) `include_local_lib`: Edit the dependency name, include path, and messages. Optionally repeat the code in this snippet for multiple libraries.
- `compile_lib`: Nothing to edit here.
- (OPTIONAL) `link_private`: Edit the library name(s)
- (OPTIONAL, non-ROS only) `link_public`: Edit the library name(s).
  For every public library you link, you need to to add the requisite `find_package(library_name REQUIRED)` lines at the start of `config.cmake.in` - see `hi_can_raw` for an example of this.
  For ROS libraries, ament handles fixes for this behind the scenes so you don't need to do anything extra.
- (OPTIONAL, ROS only) `ros_link_public`: If the project includes possibly local dependencies (`include_local_lib`), add an extra `ament_target_dependencies` line which includes those libraries specifically.
- `install_lib`: Nothing to edit here.
- (OPTIONAL, plugins only): `pluginlib_export`: Edit the plugin base type if needed.
- (SEMI-OPTIONAL) `tests`: Edit the testing targets and files.
- (ROS only): `ros_finalisation`: Nothing to edit here, just ament packaging.
