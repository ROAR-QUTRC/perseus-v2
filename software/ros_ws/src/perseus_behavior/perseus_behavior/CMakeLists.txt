cmake_minimum_required(VERSION 3.8)
project(perseus_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(THIS_PACKAGE_DEPS behaviortree_ros2 std_msgs btcpp_ros2_interfaces)

# ================================
# Add Execs
# ================================
set(EXEC1 perseus_bt_exec)
add_executable(${EXEC1} src/${EXEC1}.cpp)
ament_target_dependencies(${EXEC1} ${THIS_PACKAGE_DEPS})

# ================================
# Add Plugins
# ================================

add_library(example_plugin SHARED src/example_plugin.cpp)
target_compile_definitions(example_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(example_plugin ${THIS_PACKAGE_DEPS})

# For the tree node generation for groot2 visualization
add_executable(gen_bt_nodes src/generate_tree_nodes.cpp src/example_plugin.cpp)
ament_target_dependencies(gen_bt_nodes ${THIS_PACKAGE_DEPS})

# ================================
# Install execs
# ================================
install(TARGETS ${EXEC1} example_plugin gen_bt_nodes
        DESTINATION lib/${PROJECT_NAME})

# ================================
# INSTALL plugins for other packages to load
# ================================

install(
  TARGETS example_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins)

# ================================
# Install BT.xmls
# ================================
install(DIRECTORY behavior_trees config launch
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
