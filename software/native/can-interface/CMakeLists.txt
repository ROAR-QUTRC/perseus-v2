cmake_minimum_required(VERSION 3.5)

# Set the project name and version
project(can_interface VERSION 0.0.1 LANGUAGES CXX)

# credit https://www.kitware.com/cmake-and-the-default-build-type/
set(default_build_type "Debug")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Add -Werror flag for release builds
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Werror")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find all source files in the src directory
file(GLOB_RECURSE CODE_SOURCES src/*.cpp)

# Add the target, using the sources found
add_executable(${PROJECT_NAME} ${CODE_SOURCES})

# Set the include directories for the library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Install the executable
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

# Versioning
set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
)

# check if hi_can_lib is already present (Nix build), if not, add its subdirectory
find_package(hi_can_lib QUIET)
if(NOT hi_can_lib_FOUND)
    message(STATUS "hi_can_lib not found, adding subdirectory...")
    # add the directory with its CMakeLists.txt
    add_subdirectory(../../shared/hi-can-lib hi-can-lib)
endif()

# Link in libraries needed
target_link_libraries(${PROJECT_NAME} PRIVATE hi_can_lib)