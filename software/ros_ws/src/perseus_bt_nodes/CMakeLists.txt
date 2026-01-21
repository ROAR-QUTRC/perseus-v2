cmake_minimum_required(VERSION 3.8)
project(perseus_bt_nodes)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/register_nodes.cpp
)

find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)

ament_target_dependencies(${PROJECT_NAME}
  behaviortree_cpp
  geometry_msgs
  builtin_interfaces
  rclcpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
