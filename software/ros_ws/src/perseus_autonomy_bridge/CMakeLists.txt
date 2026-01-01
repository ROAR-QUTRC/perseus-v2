cmake_minimum_required(VERSION 3.8)
project(perseus_autonomy_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/RunWaypoints.srv"
                           "srv/CancelWaypoints.srv")

ament_export_dependencies(rosidl_default_runtime)

add_executable(nav2_waypoints_bridge src/nav2_waypoints_bridge.cpp)
ament_target_dependencies(nav2_waypoints_bridge rclcpp rclcpp_action nav2_msgs
                          geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")
target_link_libraries(nav2_waypoints_bridge ${cpp_typesupport_target})

install(TARGETS nav2_waypoints_bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY srv DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
