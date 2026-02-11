# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  perseus_sensors
  VERSION 0.0.1
  LANGUAGES CXX)
cmake_policy(SET CMP0167 NEW)
# credit https://www.kitware.com/cmake-and-the-default-build-type/
set(default_build_type "Debug")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(
    STATUS
      "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "${default_build_type}"
      CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()
# we always want debug info for stack tracing, so switch to RelWithDebInfo from
# Release
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()
# Add -Werror flag for release builds
if(CMAKE_BUILD_TYPE MATCHES "Rel.*")
  add_compile_options(-Werror)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# INCLUDE DEPENDENCIES

# public dependencies (which should propagate)
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    sensor_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
    OpenSSL
    nlohmann_json)
# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(simple_networking QUIET)
if(NOT simple_networking_FOUND)
  message(STATUS "simple_networking not found, adding subdirectory...")
  message(
    STATUS
      "Attempting to add simple_networking from: ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/simple-networking"
  )
  add_subdirectory(../../../shared/simple-networking simple-networking)
endif()

# fd_wrapper is already built as part of simple-networking
# No need to add it separately

# COMPILE (Executables)

# Find all source files in the src directory
file(GLOB_RECURSE M2M2_LIDAR_CODE_SOURCES src/m2m2_lidar/*.cpp)
# Add the target with those sources

# Add exec target
add_executable(m2m2_lidar ${M2M2_LIDAR_CODE_SOURCES})

# Add include dirs for exec
target_include_directories(m2m2_lidar
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add dependencies for the executable
ament_target_dependencies(m2m2_lidar PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(m2m2_lidar PUBLIC simple_networking OpenSSL::Crypto)

# I2C IMU Driver
# Explicit source list - build system tracks dependencies correctly
add_executable(
  i2c_imu_node
  i2c_imu_driver/src/main.cpp i2c_imu_driver/src/i2c_device.cpp
  i2c_imu_driver/src/i2c_imu_node.cpp i2c_imu_driver/src/imu_device_config.cpp)

target_include_directories(
  i2c_imu_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/i2c_imu_driver/include)

ament_target_dependencies(i2c_imu_node PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(i2c_imu_node PUBLIC fd_wrapper)

# Install ROS Executables
install(TARGETS m2m2_lidar i2c_imu_node DESTINATION lib/${PROJECT_NAME})

# Install launch
install(
  DIRECTORY launch config i2c_imu_driver/launch i2c_imu_driver/config
            i2c_imu_driver/urdf
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# ROS LIBRARY POST-INSTALL
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ROS FINALISATION
ament_package()
