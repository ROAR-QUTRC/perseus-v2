cmake_minimum_required(VERSION 3.14)
project(network_recovery)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(perseus_interfaces REQUIRED)

add_executable(
  network_recovery_node src/network_recovery_node.cpp src/network_monitor.cpp
                        src/heatmap.cpp src/recovery_navigator.cpp)

target_include_directories(
  network_recovery_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

ament_target_dependencies(
  network_recovery_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  builtin_interfaces
  tf2_ros
  tf2
  perseus_interfaces)

install(TARGETS network_recovery_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
