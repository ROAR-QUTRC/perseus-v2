cmake_minimum_required(VERSION 3.8)
project(perseus_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(aruco_detector_node src/aruco_detector_node.cpp)

ament_target_dependencies(
  aruco_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2
  geometry_msgs
  tf2_geometry_msgs)

target_include_directories(aruco_detector_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aruco_detector_node ${OpenCV_LIBS})

install(TARGETS aruco_detector_node DESTINATION lib/${PROJECT_NAME})

ament_package()
