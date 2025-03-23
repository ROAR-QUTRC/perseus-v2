# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  crc
  VERSION 0.0.1
  LANGUAGES C CXX)

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

# COMPILE (Libraries)

# Find all source files in the src directory
file(GLOB_RECURSE CODE_SOURCES src/*.c src/*.cpp)
# Add the target with those sources
add_library(${PROJECT_NAME} SHARED ${CODE_SOURCES})
# Set the include directories for the library
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# INSTALL (Libraries)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# CMake packaging - see https://blog.vito.nyc/posts/cmake-pkg/ for a good
# explanation
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.cmake.in
               ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake @ONLY)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  COMPATIBILITY SameMinorVersion)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

# Headers
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Versioning
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
# The main install - default locations are fine
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE
  LIBRARY
  RUNTIME)

# The last of the CMake packaging info
install(EXPORT ${PROJECT_NAME}_targets
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
