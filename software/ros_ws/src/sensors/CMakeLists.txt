cmake_minimum_required(VERSION 3.8)
project(sensors)

# Default to C++17 if(NOT CMAKE_CXX_STANDARD) set(CMAKE_CXX_STANDARD 17) endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Add -Werror flag for release builds
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Werror")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenSSL REQUIRED)

# Include directories
include_directories(include)

# Add the m2m2_lidar executable
add_executable(m2m2_lidar src/m2m2_lidar/m2m2_lidar.cpp)

# add dependencies
ament_target_dependencies(m2m2_lidar rclcpp sensor_msgs std_msgs)

# Link OpenSSL libraries
target_link_libraries(m2m2_lidar OpenSSL::SSL OpenSSL::Crypto)

# Install targets
install(TARGETS m2m2_lidar DESTINATION lib/${PROJECT_NAME})

# Install include directory
install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
