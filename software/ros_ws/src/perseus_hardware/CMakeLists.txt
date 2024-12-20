# PROJECT SETUP
cmake_minimum_required(VERSION 3.16)

project(
  perseus_hardware
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
set(THIS_PACKAGE_INCLUDE_DEPENDS hardware_interface pluginlib rclcpp
                                 rclcpp_lifecycle)
# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# INCLUDE POSSIBLY LOCAL DEPENDENCIES

# check if libraries are already present (Nix build), if not, add them

find_package(hi_can_raw QUIET)
if(NOT hi_can_raw_FOUND)
  message(STATUS "hi_can_raw not found, adding subdirectory...")
  # add the directory with its CMakeLists.txt
  add_subdirectory(../../../shared/hi-can-raw hi-can-raw)
  # else()
endif()

# COMPILE (Libraries)

# Find all source files in the src directory
file(GLOB_RECURSE CODE_SOURCES src/*.cpp)
# Add the target with those sources
add_library(${PROJECT_NAME} SHARED ${CODE_SOURCES})
# Set the include directories for the library
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/priv_include>)

target_link_libraries(${PROJECT_NAME} PUBLIC hi_can)
target_link_libraries(${PROJECT_NAME} PRIVATE hi_can_raw)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# INSTALL (Libraries)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# Headers
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
# CMake configuration
install(
  EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}-config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME})
# Versioning
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION}
                                                 SOVERSION 1)

# EXPORT PLUGINLIB INFORMATION

# ROS2 Control hardware interface plugin
pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)

# ROS LIBRARY POST-INSTALL
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ROS FINALISATION
ament_package()
