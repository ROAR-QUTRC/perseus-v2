# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  global_traversability
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

# COMPILER SETUP
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
#
# Deliberately grid_map_core only, not grid_map_ros: its own exported dependency is Eigen3
# alone, so it can't be what pulls in the VTK/Qt/OpenCV chain the find_package(PCL) ordering
# comment below is about. Depending on grid_map_ros would have added another, separate such
# chain (via grid_map_cv -> filters) for no benefit -- GridMap -> OccupancyGrid is hand-rolled
# below using grid_map_core's own getIndex() instead of grid_map_ros's converter.
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp sensor_msgs nav_msgs grid_map_core
                                 pcl_conversions)

# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Deliberately found before pcl_conversions below, and scoped to COMPONENTS common: found
# this way first, the cached result is a minimal PCL (no visualization/VTK/Qt). Found only via
# pcl_conversions' own transitive re-export instead, CMake resolves it unscoped, pulling all
# of PCL including visualization -- which is what actually injects "-isystem /usr/include" in
# the first place (via one of VTK/Qt's legacy CMake modules), not grid_map_core.
find_package(PCL REQUIRED COMPONENTS common)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# COMPILE (Executables)
add_executable(
  global_traversability
  src/global_traversability/global_traversability.cpp
  src/global_traversability/global_traversability_node.cpp)

target_include_directories(global_traversability
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(global_traversability PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(global_traversability PUBLIC ${PCL_LIBRARIES})

# INSTALL (ROS)
install(TARGETS global_traversability DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ROS FINALISATION
ament_package()
