# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  hi_can
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

# COMPILE (Libraries)

# Find all source files in the src directory
file(GLOB_RECURSE CODE_SOURCES src/*.cpp)
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

# TESTING SETUP (Libraries)
if(NOT DEFINED BUILD_TESTING)
  option(BUILD_TESTING "Build tests" ON)
endif()
if(NOT DEFINED TEST_COVERAGE)
  # only enable coverage if we're building tests and it's a debug build
  set(DEFAULT_TEST_COVERAGE OFF)
  if(BUILD_TESTING AND CMAKE_BUILD_TYPE MATCHES "Debug")
    set(DEFAULT_TEST_COVERAGE ON)
  endif()
  option(TEST_COVERAGE "Enable test coverage" ${DEFAULT_TEST_COVERAGE})
endif()

function(add_library_test TEST_NAME)
  cmake_parse_arguments(PARSE_ARGV 1 ARGS "EXCLUDE_DEFAULT" ""
                        "SOURCE_FILES;LINK_LIBRARIES;INCLUDE_DIRECTORIES")
  set(TEST_TARGET_NAME ${TEST_NAME}_tests)
  if(NOT ARGS_EXCLUDE_DEFAULT)
    list(APPEND ARGS_SOURCE_FILES tests/${TEST_NAME}.cpp)
  endif()
  add_executable(${TEST_TARGET_NAME} ${ARGS_SOURCE_FILES}
                                     ${ARGS_UNPARSED_ARGUMENTS})
  target_link_libraries(${TEST_TARGET_NAME} GTest::gtest_main ${PROJECT_NAME}
                        ${ARGS_LINK_LIBRARIES})
  target_include_directories(${TEST_TARGET_NAME}
                             PRIVATE ${ARGS_INCLUDE_DIRECTORIES})
  if(TEST_COVERAGE)
    target_compile_options(${TEST_TARGET_NAME} PRIVATE --coverage -O0 -g
                                                       -Wno-cpp)
    target_link_options(${TEST_TARGET_NAME} PRIVATE --coverage)
  endif()
  gtest_discover_tests(${TEST_TARGET_NAME}
                       WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endfunction()

if(BUILD_TESTING)
  enable_testing()

  find_package(GTest REQUIRED)
  include(GoogleTest)

  if(TEST_COVERAGE)
    target_compile_options(
      ${PROJECT_NAME} PRIVATE --coverage -O0 -g -fkeep-inline-functions
                              -Wno-cpp)
    target_link_options(${PROJECT_NAME} PRIVATE --coverage)
  endif()

  add_library_test(addresses)
  add_library_test(filter)
  add_library_test(packet_manager)
  add_library_test(packet)
endif()
