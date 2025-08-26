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
find_package(ament_index_cpp REQUIRED)

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

# Add the executable for the cube_detector node
add_executable(cube_detector src/cube_detector.cpp)

# Set C++ standard
set_target_properties(cube_detector PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

target_include_directories(cube_detector PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  /nix/store/adba59jwdazrgsd5ff7y5qjiim3v1hqh-onnxruntime-1.22.0-dev/include
)

target_link_directories(cube_detector PUBLIC
  /nix/store/5a03r2hnp3kpnjmv7ifnfhi38fd81kar-onnxruntime-1.22.0/lib
)

ament_target_dependencies(
  cube_detector
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2
  geometry_msgs
  tf2_geometry_msgs
  ament_index_cpp
)

target_link_libraries(detector_node aruco_detector_lib)

# Link cube_detector with specific linker flags to handle ABI compatibility
target_link_libraries(cube_detector aruco_detector_lib onnxruntime)
target_link_options(cube_detector PRIVATE "-Wl,--allow-multiple-definition" "-Wl,--unresolved-symbols=ignore-in-shared-libs")

# Install targets
install(TARGETS aruco_detector_lib detector_node cube_detector
        DESTINATION lib/${PROJECT_NAME})

# Install include directory
install(DIRECTORY include/ DESTINATION include/)

# Install additional resources (if any)
install(DIRECTORY yolo_model DESTINATION share/${PROJECT_NAME})

ament_package()
