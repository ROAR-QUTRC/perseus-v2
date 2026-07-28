cmake_minimum_required(VERSION 3.10)
project(perseus_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS + system dependencies ─────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
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
find_package(rclcpp_components REQUIRED)

# ── ONNX Runtime (via pkg-config — works with Nix) ────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(ORT REQUIRED libonnxruntime)

message(STATUS "ONNXRuntime include: ${ORT_INCLUDE_DIRS}")
message(STATUS "ONNXRuntime lib:     ${ORT_LIBRARIES}")

include_directories(include)

# ── component library ─────────────────────────────────────────────────────────
# All three nodes are built as rclcpp components so they can be loaded into a
# single container. With intra-process comms enabled the container passes the
# camera image between them by pointer instead of copying and serialising it.
add_library(
  perseus_vision_components SHARED
  src/common/detection_renderer.cpp src/aruco_detector/aruco_detector.cpp
  src/cube_detector/cube_detector.cpp
  src/detection_overlay/detection_overlay.cpp)

target_include_directories(
  perseus_vision_components
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PRIVATE ${ORT_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_link_libraries(perseus_vision_components ${ORT_LIBRARIES} ${OpenCV_LIBS})

ament_target_dependencies(
  perseus_vision_components
  rclcpp
  rclcpp_components
  rcl_interfaces
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
  visualization_msgs
  perseus_interfaces)

rclcpp_components_register_nodes(
  perseus_vision_components "perseus_vision::ArucoDetector"
  "perseus_vision::CubeDetector" "perseus_vision::DetectionOverlay")

# ── standalone executables ────────────────────────────────────────────────────
# Thin wrappers so each node can still be run on its own with `ros2 run`.
add_executable(aruco_detector_node src/aruco_detector/aruco_detector_node.cpp)
target_link_libraries(aruco_detector_node perseus_vision_components)
ament_target_dependencies(aruco_detector_node rclcpp)

add_executable(cube_detector src/cube_detector/cube_detector_node.cpp)
target_link_libraries(cube_detector perseus_vision_components)
ament_target_dependencies(cube_detector rclcpp)

add_executable(detection_overlay_node
               src/detection_overlay/detection_overlay_node.cpp)
target_link_libraries(detection_overlay_node perseus_vision_components)
ament_target_dependencies(detection_overlay_node rclcpp)

# ── install ───────────────────────────────────────────────────────────────────
install(
  TARGETS perseus_vision_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS aruco_detector_node cube_detector detection_overlay_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

# ── ONNX Model Download ──────────────────────────────────────────────────────
set(MODEL_FILENAME "cube_detector_yolob8s.onnx")
set(MODEL_URL
    "https://github.com/ROAR-QUTRC/perseus-v2/releases/download/models-v1/${MODEL_FILENAME}"
)
set(MODEL_SHA256
    "1245a1285628839cac4a86dc1380c55b64bc8f764b4190c1c6e7162dafca55f7")
set(MODEL_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/models")
set(MODEL_OUTPUT_PATH "${MODEL_OUTPUT_DIR}/${MODEL_FILENAME}")

# Check if model was provided externally (e.g., by Nix fetchurl)
set(MODEL_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/models/${MODEL_FILENAME}")

if(EXISTS "${MODEL_SOURCE_PATH}")
  message(STATUS "[perseus_vision] Using pre-provided model from source tree.")
  set(MODEL_OUTPUT_PATH "${MODEL_SOURCE_PATH}")
elseif(NOT EXISTS "${MODEL_OUTPUT_PATH}")
  file(MAKE_DIRECTORY "${MODEL_OUTPUT_DIR}")
  message(STATUS "[perseus_vision] Downloading ONNX model...")
  file(
    DOWNLOAD "${MODEL_URL}" "${MODEL_OUTPUT_PATH}"
    EXPECTED_HASH SHA256=${MODEL_SHA256}
    SHOW_PROGRESS
    STATUS DOWNLOAD_STATUS)
  list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
  if(NOT STATUS_CODE EQUAL 0)
    message(
      FATAL_ERROR "[perseus_vision] Model download failed: ${DOWNLOAD_STATUS}")
  endif()
  message(STATUS "[perseus_vision] Model downloaded successfully.")
endif()

install(FILES "${MODEL_OUTPUT_PATH}" DESTINATION share/${PROJECT_NAME}/models)
# ─────────────────────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
