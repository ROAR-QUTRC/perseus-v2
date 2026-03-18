cmake_minimum_required(VERSION 3.10)
project(perseus_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS + system dependencies ─────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(perseus_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── ONNX Runtime (via pkg-config — works with Nix) ────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(ORT REQUIRED libonnxruntime)

message(STATUS "ONNXRuntime include: ${ORT_INCLUDE_DIRS}")
message(STATUS "ONNXRuntime lib:     ${ORT_LIBRARIES}")

include_directories(include)

# ── aruco_detector_node ───────────────────────────────────────────────────────
add_executable(aruco_detector_node src/aruco_detector.cpp
                                   src/aruco_detector_node.cpp)

ament_target_dependencies(
  aruco_detector_node
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
  perseus_interfaces)

target_include_directories(
  aruco_detector_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# ── cube_detector ─────────────────────────────────────────────────────────────
add_executable(cube_detector src/cube_detector.cpp src/cube_detector_node.cpp)

target_include_directories(
  cube_detector
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PRIVATE ${ORT_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_link_libraries(cube_detector ${ORT_LIBRARIES} ${OpenCV_LIBS})

ament_target_dependencies(
  cube_detector
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  ament_index_cpp
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs)

# ── install ───────────────────────────────────────────────────────────────────
install(TARGETS aruco_detector_node cube_detector
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
