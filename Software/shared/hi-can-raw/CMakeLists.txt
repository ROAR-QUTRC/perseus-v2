cmake_minimum_required(VERSION 3.5)

# Set the project name and version
project(hi_can_raw VERSION 0.0.1 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find all source files in the src directory
file(GLOB_RECURSE CODE_SOURCES src/*.cpp)

# Add the target, using the sources found
add_library(${PROJECT_NAME} SHARED ${CODE_SOURCES})

# Set the include directories for the library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the library files
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the headers
install(DIRECTORY include/
  DESTINATION include
)

# Export the CMake configuration so other CMake projects can import it
install(EXPORT ${PROJECT_NAME}_export
  FILE ${PROJECT_NAME}-config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# Versioning
set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
)

# check if hi_can_lib is already present (Nix build), if not, add its subdirectory
find_package(hi_can_lib QUIET)
if(NOT hi_can_lib_FOUND)
    message(STATUS "hi_can_lib not found, adding subdirectory...")
    # add the directory with its CMakeLists.txt
    add_subdirectory(../hi-can-lib)
endif()

# Link in libraries needed
target_link_libraries(${PROJECT_NAME} PUBLIC hi_can_lib)