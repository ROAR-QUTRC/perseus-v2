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

# Address CMP0167 warning
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# INCLUDE DEPENDENCIES Required external packages
find_package(Boost REQUIRED COMPONENTS system)
find_package(Curses REQUIRED)
find_package(yaml-cpp REQUIRED)

# For finding or adding simple_networking
find_package(simple_networking QUIET)
if(NOT simple_networking_FOUND)
  message(STATUS "simple_networking not found, adding subdirectory...")
  message(
    STATUS
      "Attempting to add simple_networking from: ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/simple-networking"
  )
  add_subdirectory(../shared/simple-networking simple-networking)
endif()

# COMPILE (Libraries) Common library for shared functionality
add_library(
  arm_teleop_common STATIC
  src/network_protocol.cpp src/networked_arm_controller.cpp
  src/leader_controller.cpp src/follower_controller.cpp)

# Set include directories for the library
target_include_directories(
  arm_teleop_common
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# Link dependencies
target_link_libraries(
  arm_teleop_common
  PUBLIC simple_networking
  PRIVATE Boost::system
  PRIVATE ${CURSES_LIBRARIES}
  PRIVATE yaml-cpp
  PRIVATE pthread)

# COMPILE (Executables) Leader executable
add_executable(arm-teleop-leader src/leader_main.cpp)
target_include_directories(
  arm-teleop-leader
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(
  arm-teleop-leader
  PRIVATE arm_teleop_common
  PRIVATE Boost::system
  PRIVATE ${CURSES_LIBRARIES}
  PRIVATE yaml-cpp
  PRIVATE pthread)

# Follower executable
add_executable(arm-teleop-follower src/follower_main.cpp)
target_include_directories(
  arm-teleop-follower
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(
  arm-teleop-follower
  PRIVATE arm_teleop_common
  PRIVATE Boost::system
  PRIVATE ${CURSES_LIBRARIES}
  PRIVATE yaml-cpp
  PRIVATE pthread)

# INSTALL (Executables)
install(TARGETS arm-teleop-leader arm-teleop-follower RUNTIME DESTINATION bin)
# Versioning
set_target_properties(arm-teleop-leader PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(arm-teleop-follower PROPERTIES VERSION ${PROJECT_VERSION})
