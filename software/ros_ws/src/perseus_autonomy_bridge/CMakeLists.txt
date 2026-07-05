cmake_minimum_required(VERSION 3.10)
project(perseus_autonomy_bridge)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(perseus_interfaces REQUIRED)

add_executable(nav2_waypoints_bridge src/nav2_waypoints_bridge.cpp)
ament_target_dependencies(nav2_waypoints_bridge rclcpp rclcpp_action nav2_msgs
                          geometry_msgs perseus_interfaces)

install(TARGETS nav2_waypoints_bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
