cmake_minimum_required(VERSION 3.8)
project(perseus_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
set(onnxruntime_DIR "/nix/store/nh5125kw87mycaq14vdi02kzmnr49bqn-onnxruntime-1.18.1-dev/lib/cmake/onnxruntime")
find_package(onnxruntime REQUIRED)
# Add the library for the ArucoDetector class
add_library(aruco_detector_lib src/aruco_detector.cpp)

target_include_directories(aruco_detector_lib PUBLIC include
                                                     ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(
  aruco_detector_lib
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2
  geometry_msgs
  tf2_geometry_msgs)

target_link_libraries(aruco_detector_lib ${OpenCV_LIBS})

# Add the executable for the detector node
add_executable(detector_node src/detector_node.cpp)

target_include_directories(detector_node PUBLIC include)

ament_target_dependencies(
  detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2
  geometry_msgs
  tf2_geometry_msgs)

target_link_libraries(detector_node aruco_detector_lib)

# Install targets
install(TARGETS aruco_detector_lib detector_node
        DESTINATION lib/${PROJECT_NAME})

# Install include directory
install(DIRECTORY include/ DESTINATION include/)

# Install additional resources (if any)
install(DIRECTORY yolo_model DESTINATION share/${PROJECT_NAME})

ament_package()
