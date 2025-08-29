# PROJECT SETUP Using CMake 3.20 for better modern CMake features and Boost
# integration
cmake_minimum_required(VERSION 3.20)
project(
  perseus_lite_hardware
  VERSION 0.0.1
  LANGUAGES CXX)

# Version compatibility check for better Boost integration
if(CMAKE_VERSION VERSION_LESS "3.20.0")
  message(
    WARNING
      "Consider updating CMake to version 3.20 or higher for better Boost integration"
  )
endif()

# Build type configuration Default to Debug build if not specified, with
# RelWithDebInfo for Release builds
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

# Always include debug info for better stack traces
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Compiler warning configuration
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Treat warnings as errors in Release builds
if(CMAKE_BUILD_TYPE MATCHES "Rel.*")
  add_compile_options(-Werror)
endif()

# Set C++20 as the required standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define package dependencies ROS2 package dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS hardware_interface pluginlib rclcpp
                                 rclcpp_lifecycle)
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

# Boost configuration with explicit settings for better control
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.71.0 REQUIRED COMPONENTS system)

# Find all required build dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Find all required package dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Collect all source files recursively from src directory
file(GLOB_RECURSE CODE_SOURCES src/*.cpp)

# Define the shared library target
add_library(${PROJECT_NAME} SHARED ${CODE_SOURCES})

# Configure include directories with proper visibility
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC # Headers for clients of this library
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE # Headers only for internal implementation
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

# Link against dependencies using explicit targets This ensures proper linking
# and dependency propagation
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC # Boost libraries
         Boost::system
         # ROS2 libraries with explicit targets
         hardware_interface::hardware_interface
         pluginlib::pluginlib
         rclcpp::rclcpp
         rclcpp_lifecycle::rclcpp_lifecycle)

# Installation configuration Install library binaries
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install header files
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Install CMake config files
install(
  EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}-config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME})

# Set library properties including version information
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION}
                                                 SOVERSION 1)

# ROS2 specific configuration Export plugin description
pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)

# Export targets and dependencies for downstream packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Finalize the package
ament_package()
