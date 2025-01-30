cmake_minimum_required(VERSION 3.8)
project(perseus_navigator)

# Enable all warnings for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_common REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(perseus_msgs REQUIRED) # Ensure perseus_msgs is found

# ==================================
# Create the Plugin Library
# ==================================
add_library(
  ${PROJECT_NAME} SHARED src/perseus_navigator.cpp # Ensure this is the correct
                                                   # implementation file
)

# Ensure the correct include directories are used
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp> # For
                                                                        # generated
                                                                        # action
                                                                        # headers
)

# ✅ Use ONLY `ament_target_dependencies()`, no `target_link_libraries()`
ament_target_dependencies(
  ${PROJECT_NAME}
  geometry_msgs
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  tf2_ros
  perseus_msgs # Ensure perseus_msgs is properly linked
  rosidl_runtime_cpp # Needed for message types
)

# ==================================
# Install BT.xmls, launch, and params
# ==================================
install(DIRECTORY behavior_trees launch params
        DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# ==================================
# Install and Export the Plugin
# ==================================
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Plugin description file
pluginlib_export_plugin_description_file(nav2_core navigator_plugin.xml)

# Export libraries and dependencies
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  tf2_ros
  perseus_msgs
  rosidl_runtime_cpp)

ament_export_targets(export_${PROJECT_NAME})

# Finalize package
ament_package()
