cmake_minimum_required(VERSION 3.23)

project(
  perseus_simulation
  VERSION 0.0.1
  LANGUAGES CXX)

# Build type setup
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
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(CMAKE_BUILD_TYPE MATCHES "Rel.*")
  add_compile_options(-Werror)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# From 15-include_dependencies.cmake
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    gazebo_ros
    gazebo_ros2_control
    controller_interface
    hardware_interface
    pluginlib
    perseus_description
    perseus_hardware)

set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# From 21-compile_lib.cmake (if we end up needing custom gazebo plugins)
file(GLOB_RECURSE CODE_SOURCES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${CODE_SOURCES})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# From 23-ros_link_public.cmake
ament_target_dependencies(${PROJECT_NAME} PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# From 72-install_lib.cmake
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.cmake.in
               ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake @ONLY)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  COMPATIBILITY SameMinorVersion)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE
  LIBRARY
  RUNTIME)

install(EXPORT ${PROJECT_NAME}_targets
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

# From 73-install_ros_bringup.cmake (modified for simulation files)
install(DIRECTORY launch worlds models config DESTINATION share/${PROJECT_NAME})

# From 74-pluginlib_export.cmake (if we have custom plugins)
pluginlib_export_plugin_description_file(gazebo_ros ${PROJECT_NAME}.xml)

# From 80-ros_post_install_lib.cmake
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# From 90-ros_tests.cmake (if we have tests)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test test/test_simulation.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

# From 95-ros_finalisation.cmake
ament_package()
