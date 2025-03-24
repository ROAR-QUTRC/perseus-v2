# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  arm-teleop-network
  VERSION 0.1.0
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
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp)
# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Find other required packages
find_package(Boost REQUIRED COMPONENTS system)
find_package(Curses REQUIRED)
find_package(yaml-cpp REQUIRED)

# Check if simple_networking is available
find_package(simple_networking QUIET)
if(NOT simple_networking_FOUND)
  message(STATUS "simple_networking not found, adding subdirectory...")
  # Add the directory with its CMakeLists.txt
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../shared/simple-networking
                   simple-networking)
endif()

# COMPILE (Protocol Library)

# Protocol library
add_library(arm-servo-protocol src/arm-servo-protocol.cpp)

target_include_directories(
  arm-servo-protocol
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# Network interface library
add_library(arm-network src/arm-network.cpp)

target_include_directories(
  arm-network PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(
  arm-network
  PUBLIC arm-servo-protocol
  PUBLIC simple_networking
  PUBLIC Boost::system)

# COMPILE (Executables)

# Leader Arm Executable
add_executable(arm-teleop-leader src/leader-main.cpp src/perseus-arm-teleop.cpp)

target_include_directories(
  arm-teleop-leader
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC ${CURSES_INCLUDE_DIR}
  PUBLIC ${YAML_CPP_INCLUDE_DIR})

target_link_libraries(
  arm-teleop-leader
  PRIVATE arm-servo-protocol
  PRIVATE arm-network
  PRIVATE Boost::system
  PRIVATE ${CURSES_LIBRARIES}
  PRIVATE yaml-cpp
  PRIVATE pthread)

# Follower Arm Executable
add_executable(arm-teleop-follower src/follower-main.cpp
                                   src/perseus-arm-teleop.cpp)

target_include_directories(
  arm-teleop-follower
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC ${CURSES_INCLUDE_DIR}
  PUBLIC ${YAML_CPP_INCLUDE_DIR})

target_link_libraries(
  arm-teleop-follower
  PRIVATE arm-servo-protocol
  PRIVATE arm-network
  PRIVATE Boost::system
  PRIVATE ${CURSES_LIBRARIES}
  PRIVATE yaml-cpp
  PRIVATE pthread)

# INSTALL (Executables)
install(TARGETS arm-teleop-leader arm-teleop-follower RUNTIME DESTINATION bin)
# Versioning
set_target_properties(arm-teleop-leader PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(arm-teleop-follower PROPERTIES VERSION ${PROJECT_VERSION})
