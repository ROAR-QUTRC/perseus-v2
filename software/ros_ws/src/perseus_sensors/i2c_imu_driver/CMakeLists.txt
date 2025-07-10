# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  i2c_imu_driver
  VERSION 0.0.1
  LANGUAGES CXX)

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
  add_compile_options(-Wall -Wextra -Wpedantic)
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
    tf2_geometry_msgs)

# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Find fd-wrapper shared library
find_package(fd-wrapper QUIET)
if(NOT fd-wrapper_FOUND)
  message(STATUS "fd-wrapper not found, adding subdirectory...")
  message(
    STATUS
      "Attempting to add fd-wrapper from: ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/fd-wrapper"
  )
  add_subdirectory(../../../shared/fd-wrapper fd-wrapper)
endif()

# COMPILE (Executables and Libraries)

# Find all source files
file(GLOB_RECURSE I2C_IMU_SOURCES src/*.cpp)

# Add executable target
add_executable(i2c_imu_node ${I2C_IMU_SOURCES})

# Add include directories
target_include_directories(i2c_imu_node
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add dependencies for the executable
ament_target_dependencies(i2c_imu_node PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(i2c_imu_node PUBLIC fd-wrapper)

# Install ROS Executable
install(TARGETS i2c_imu_node DESTINATION lib/${PROJECT_NAME})

# Install launch, config, and urdf files
install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# ROS LIBRARY POST-INSTALL
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ROS FINALISATION
ament_package()