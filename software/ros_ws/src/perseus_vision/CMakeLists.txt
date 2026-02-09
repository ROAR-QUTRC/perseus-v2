<<<<<<< HEAD
cmake_minimum_required(VERSION 3.10)
=======
cmake_minimum_required(VERSION 3.8)
>>>>>>> 22261573 (feat: add ArUco detection capabilities with configuration and node setup)
project(perseus_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
<<<<<<< HEAD
<<<<<<< HEAD
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(perseus_interfaces REQUIRED)

include_directories(include)

add_executable(aruco_detector_node src/aruco_detector.cpp
                                   src/aruco_detector_node.cpp)

ament_target_dependencies(
  aruco_detector_node
=======
find_package(perseus_interfaces REQUIRED)
=======
>>>>>>> 2c48d33b (fix: correct coordinate adjustment for marker pose in ArucoDetector)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(perseus_interfaces REQUIRED)

include_directories(include)

add_executable(aruco_detector src/aruco_detector.cpp
                                   src/aruco_detector_node.cpp)

ament_target_dependencies(
  aruco_detector
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
  visualization_msgs
  vision_msgs
  perseus_interfaces)

target_include_directories(
  aruco_detector
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

install(TARGETS aruco_detector DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
