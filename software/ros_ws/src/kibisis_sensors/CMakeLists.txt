cmake_minimum_required(VERSION 3.20)
project(
  kibisis_sensors
  VERSION 0.1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_executable(kibisis_sensors_node src/kibisis_sensors_node.cpp)

ament_target_dependencies(kibisis_sensors_node rclcpp std_msgs std_srvs
                          controller_manager_msgs)

install(TARGETS kibisis_sensors_node DESTINATION lib/${PROJECT_NAME})

ament_package()
