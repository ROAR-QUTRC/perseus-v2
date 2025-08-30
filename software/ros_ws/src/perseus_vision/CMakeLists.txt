cmake_minimum_required(VERSION 3.8)
project(perseus_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required ROS2 and system packages
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
find_package(image_transport REQUIRED)

# =============================
# ONNX Runtime (manual since nix)
# =============================
# Try to find ONNX Runtime using file(GLOB) to handle nix store paths
file(GLOB ONNXRUNTIME_PATHS "/nix/store/*onnxruntime*")
set(ONNXRUNTIME_INCLUDE_DIR "")
set(ONNXRUNTIME_LIB "")

foreach(ONNX_PATH ${ONNXRUNTIME_PATHS})
  if(EXISTS "${ONNX_PATH}/include/onnxruntime_cxx_api.h")
    set(ONNXRUNTIME_INCLUDE_DIR "${ONNX_PATH}/include")
  endif()
  if(EXISTS "${ONNX_PATH}/lib/libonnxruntime.so")
    set(ONNXRUNTIME_LIB "${ONNX_PATH}/lib/libonnxruntime.so")
  endif()
endforeach()

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
  message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIB}")
  message(STATUS "ONNX Runtime include: ${ONNXRUNTIME_INCLUDE_DIR}")
else()
  message(
    FATAL_ERROR "ONNX Runtime not found, please install or set paths manually")
endif()

# Add ArucoDetector library
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

# Detector node (aruco)
add_executable(aruco_detector src/detector_node.cpp)
target_link_libraries(aruco_detector aruco_detector_lib)

# Add missing ament_target_dependencies for aruco_detector
ament_target_dependencies(
  aruco_detector
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2
  geometry_msgs
  tf2_geometry_msgs)

# Cube detector node (YOLO + ONNX)
add_executable(cube_detector src/cube_detector.cpp)

set_target_properties(cube_detector PROPERTIES CXX_STANDARD 17
                                               CXX_STANDARD_REQUIRED ON)

target_include_directories(cube_detector PUBLIC ${OpenCV_INCLUDE_DIRS}
                                                ${ONNXRUNTIME_INCLUDE_DIR})

target_link_libraries(cube_detector aruco_detector_lib ${ONNXRUNTIME_LIB}
                      ${OpenCV_LIBS})

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
  image_transport)

# Linker flags for ABI stuff (if needed)
target_link_options(cube_detector PRIVATE "-Wl,--allow-multiple-definition"
                    "-Wl,--unresolved-symbols=ignore-in-shared-libs")

# =============================
# Install
# =============================
install(TARGETS aruco_detector cube_detector DESTINATION lib/${PROJECT_NAME})

install(TARGETS aruco_detector_lib DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
  OPTIONAL)

# Install launch files with OPTIONAL flag like perseus_simulation
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL)

# Install yolo_model directory if it exists
# Install the ONNX model file explicitly (if it exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/yolo_model/best.onnx")
  install(
    FILES yolo_model/best.onnx
    DESTINATION share/${PROJECT_NAME}/yolo_model
  )
endif()
ament_package()
