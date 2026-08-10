# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  perseus_sensors
  VERSION 0.0.1
  LANGUAGES CXX)
cmake_policy(SET CMP0167 NEW)
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
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp rclcpp_components sensor_msgs
                                 geometry_msgs nav_msgs)
# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE (Executables)

# Topic Remapper
# Dynamic topic remapper that detects message type and republishes at lower frequency
add_executable(topic_remapper src/topic_remapper/main.cpp
                              src/topic_remapper/topic_remapper.cpp)

target_include_directories(topic_remapper
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(topic_remapper PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install ROS Executables
install(TARGETS topic_remapper DESTINATION lib/${PROJECT_NAME})

add_library(bias_estimator_component SHARED
            src/bias_remover/bias_estimator_component.cpp)
ament_target_dependencies(bias_estimator_component rclcpp rclcpp_components
                          sensor_msgs geometry_msgs nav_msgs)
rclcpp_components_register_nodes(bias_estimator_component
                                 "imu_processors::BiasEstimator")

add_library(bias_remover_component SHARED
            src/bias_remover/bias_remover_component.cpp)
ament_target_dependencies(bias_remover_component rclcpp rclcpp_components
                          sensor_msgs geometry_msgs)
rclcpp_components_register_nodes(bias_remover_component
                                 "imu_processors::BiasRemover")

install(
  TARGETS bias_estimator_component bias_remover_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Dust Filter
# Radius outlier removal, decluttering the Livox point cloud before FAST-LIO or the
# obstacle costmap sees it. Built as a component so it can be loaded into a shared
# container with another node (e.g. FAST-LIO's LaserMappingNode) and pass point clouds
# between them by pointer instead of over DDS.
add_library(dust_filter_component SHARED src/dust_filter/dust_filter.cpp)

target_include_directories(dust_filter_component
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(dust_filter_component rclcpp rclcpp_components
                          sensor_msgs)
rclcpp_components_register_nodes(dust_filter_component
                                 "perseus_sensors::DustFilter")

install(
  TARGETS dust_filter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Thin wrapper so the filter can still be run standalone with `ros2 run`.
add_executable(dust_filter src/dust_filter/dust_filter_node.cpp)
target_link_libraries(dust_filter dust_filter_component)
ament_target_dependencies(dust_filter rclcpp)

install(TARGETS dust_filter DESTINATION lib/${PROJECT_NAME})

# Install launch
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# ROS LIBRARY POST-INSTALL
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ROS FINALISATION
ament_package()
