# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  perseus_can_if
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
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp sensor_msgs nlohmann_json)

# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(hi_can_raw QUIET)
if(NOT hi_can_raw_FOUND)
  message(STATUS "hi-can-raw not found, adding subdirectory...")
  message(
    STATUS "Attempting to add hi-can-raw from: ../../../shared/hi-can-raw")
  add_subdirectory(../../../shared/hi-can-raw hi-can-raw)
endif()

# COMPILE (Executables)

# Find all source files for the executables
file(GLOB_RECURSE RCB_DRIVER_CODE_SOURCES src/rcb_driver/*.cpp)

# Add exec targets
add_executable(rcb_driver ${RCB_DRIVER_CODE_SOURCES})

# Add include dirs
target_include_directories(rcb_driver
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add dependencies for the executables
ament_target_dependencies(rcb_driver PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(rcb_driver PUBLIC hi_can_raw)

# Install ROS Executable
install(TARGETS rcb_driver DESTINATION lib/${PROJECT_NAME})

ament_package()
