# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  as7343_driver
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
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp perseus_interfaces)
# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Resolve fd_wrapper from shared directory
find_package(fd_wrapper QUIET)
if(NOT fd_wrapper_FOUND)
  find_package(simple_networking QUIET)
  if(NOT simple_networking_FOUND)
    message(STATUS "simple_networking not found, adding subdirectory...")
    add_subdirectory(../../../shared/simple-networking simple-networking)
  endif()
endif()

# COMPILE (Executables)

add_executable(as7343_node src/main.cpp src/i2c_device.cpp
                           src/as7343_device.cpp src/as7343_node.cpp)

target_include_directories(as7343_node
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(as7343_node PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(as7343_node PUBLIC fd_wrapper)

# Install ROS Executable
install(TARGETS as7343_node DESTINATION lib/${PROJECT_NAME})

# Install launch and config
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# ROS LIBRARY POST-INSTALL
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ROS FINALISATION
ament_package()
