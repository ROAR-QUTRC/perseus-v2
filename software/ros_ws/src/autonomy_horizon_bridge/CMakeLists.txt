cmake_minimum_required(VERSION 3.8)
project(autonomy_horizon_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(pluginlib REQUIRED)
find_package(perseus_msgs REQUIRED)

add_executable(frontier_server src/frontier_server.cpp)
ament_target_dependencies(frontier_server
  rclcpp tf2_ros tf2_geometry_msgs nav_msgs geometry_msgs visualization_msgs perseus_msgs
)

install(TARGETS frontier_server
  DESTINATION lib/${PROJECT_NAME}
)
add_library(horizon_bridge_bt SHARED
  src/horizon_bridge_bt_node.cpp
)
ament_target_dependencies(horizon_bridge_bt
  rclcpp rclcpp_action nav2_behavior_tree nav2_msgs geometry_msgs pluginlib perseus_msgs
)

install(TARGETS horizon_bridge_bt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# pluginlib_export_plugin_description_file(nav2_behavior_tree plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
