# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  template_package
  VERSION 0.0.1
  LANGUAGES CXX)
set(NODE_NAMES template_node)

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
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp std_msgs)

# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# COMPILE (Executables)
foreach(node IN ITEMS ${NODE_NAMES})
  # Find all source files for the executables
  file(GLOB_RECURSE CODE_SOURCES src/${node}/*.cpp)

  # Add exec targets
  add_executable(${node} ${CODE_SOURCES})

  # Add include dirs
  target_include_directories(
    ${node}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
endforeach()


# DEPENDENCIES (executables)
foreach(node IN ITEMS ${NODE_NAMES})
  # Add dependencies for the executables
    ament_target_dependencies(
      ${node} PUBLIC
      ${THIS_PACKAGE_INCLUDE_DEPENDS})
endforeach()


# INSTALL(ROS Executables)
foreach(node IN ITEMS ${NODE_NAMES})
  # Install ROS Executable
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()


# INSTALL (ROS)
install(
  DIRECTORY config launch maps rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

  
# ROS FINALISATION
ament_package() 

