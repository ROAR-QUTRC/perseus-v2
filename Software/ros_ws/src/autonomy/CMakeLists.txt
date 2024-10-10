cmake_minimum_required(VERSION 3.8)
project(autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add the executable
add_executable(mapper src/mapper.cpp)

# Include directories
target_include_directories(mapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set C++ standard
target_compile_features(mapper PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Link against rclcpp
ament_target_dependencies(mapper rclcpp)

# Install the executable
install(TARGETS mapper
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
