# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  perseus_lite_hud
  VERSION 0.1.0
  LANGUAGES CXX)

set(default_build_type "Debug")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(
    STATUS
      "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "${default_build_type}"
      CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()
if(CMAKE_BUILD_TYPE MATCHES "Rel.*")
  add_compile_options(-Werror)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    cv_bridge
    image_transport
    tf2_ros
    tf2_geometry_msgs)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# COMPILE
add_executable(
  hud_overlay_node
  src/main.cpp
  src/hud_node.cpp
  src/hud_renderer.cpp
  src/hud_elements/bearing_compass.cpp
  src/hud_elements/pip_map.cpp
  src/hud_elements/lidar_proximity.cpp
  src/hud_elements/velocity_gauge.cpp
  src/hud_elements/odometer.cpp)

target_include_directories(hud_overlay_node
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(hud_overlay_node PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(hud_overlay_node PUBLIC opencv_core opencv_imgproc)

# INSTALL
install(TARGETS hud_overlay_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# FINALISE
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
