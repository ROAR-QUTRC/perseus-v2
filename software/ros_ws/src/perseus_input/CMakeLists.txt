# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  perseus_input
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
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp sensor_msgs geometry_msgs actuator_msgs)

# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE (Executables)

# Find all source files for the executables
file(GLOB_RECURSE GENERIC_CONTROLLER_CODE_SOURCES src/generic_controller/*.cpp)

# Add exec targets
add_executable(generic_controller ${GENERIC_CONTROLLER_CODE_SOURCES})

# Add include dirs
target_include_directories(generic_controller
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add dependencies for the executables
ament_target_dependencies(generic_controller PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install ROS Executable
install(TARGETS generic_controller DESTINATION lib/${PROJECT_NAME})

# INSTALL (ROS)
install(
  DIRECTORY config launch maps rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# TESTING
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(GTest REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

  enable_testing()

  # Test files are located in /software/tests/
  set(TESTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/perseus_input")

  # Axis math unit tests (pure logic, no ROS dependencies)
  add_executable(axis_math_test ${TESTS_DIR}/axis_math_test.cpp)
  target_link_libraries(axis_math_test GTest::gtest_main)
  target_include_directories(axis_math_test
                             PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

  # Controller config validation tests
  add_executable(controller_config_test ${TESTS_DIR}/controller_config_test.cpp)
  target_link_libraries(controller_config_test GTest::gtest_main
                        ${YAML_CPP_LIBRARIES})
  target_include_directories(controller_config_test
                             PUBLIC ${YAML_CPP_INCLUDE_DIRS})
  target_compile_options(controller_config_test PUBLIC ${YAML_CPP_CFLAGS_OTHER})

  include(GoogleTest)
  gtest_discover_tests(axis_math_test)
  gtest_discover_tests(controller_config_test)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
