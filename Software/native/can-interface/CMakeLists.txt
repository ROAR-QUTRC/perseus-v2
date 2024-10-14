cmake_minimum_required(VERSION 3.5)

# Set the project name and version
project(can_interface VERSION 0.0.1 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
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

# check if hi_can_client is already present (Nix build), if not, add its subdirectory
find_package(hi_can_client QUIET)
if(NOT hi_can_client_FOUND)
    message(STATUS "hi_can_client not found, adding subdirectory...")
    # add the directory with its CMakeLists.txt
    add_subdirectory(../../shared/hi-can-client)
endif()

# Link in libraries needed
target_link_libraries(${PROJECT_NAME} PRIVATE hi_can_client)