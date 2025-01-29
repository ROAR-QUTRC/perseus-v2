cmake_minimum_required(VERSION 3.8)
project(perseus_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# ==================================
# Create the Plugin Library
# ==================================
add_library(
  ${PROJECT_NAME} SHARED src/perseus_navigator.cpp # Change this to your actual
                                                   # implementation file
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC nav2_util::nav2_util_core pluginlib::pluginlib rclcpp::rclcpp
         rclcpp_action::rclcpp_action tf2_ros::tf2_ros)

target_link_libraries(
  ${PROJECT_NAME} PRIVATE ament_index_cpp::ament_index_cpp
                          rclcpp_lifecycle::rclcpp_lifecycle)

# ==================================
# Install BT.xmls, launch and params
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

pluginlib_export_plugin_description_file(nav2_core navigator_plugin.xml)

ament_export_libraries(${PROJECT_NAME}) # Export library for use in other
                                        # packages
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  tf2_ros)

ament_export_targets(export_${PROJECT_NAME})

ament_package()
