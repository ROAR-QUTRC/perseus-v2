cmake_minimum_required(VERSION 3.10)
project(
  perseus_interfaces
  VERSION 0.0.0
  LANGUAGES CXX C)
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
# set(THIS_PACKAGE_INCLUDE_DEPENDS )

# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake
                               rosidl_default_generators geometry_msgs)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/GetFrontierGoal.srv"
  "msg/ObjectDetections.msg"
  "srv/DetectObjects.srv"
  "msg/NavigationData.msg"
  "srv/RunWaypoints.srv"
  "msg/Waypoint.msg"
  DEPENDENCIES
  geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
