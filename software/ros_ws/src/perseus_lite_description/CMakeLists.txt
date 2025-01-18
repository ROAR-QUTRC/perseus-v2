cmake_minimum_required(VERSION 3.23)
project(perseus_lite_hardware)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add library
add_library(${PROJECT_NAME} SHARED src/st3215_system.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp
                          rclcpp_lifecycle)
target_link_libraries(${PROJECT_NAME} PUBLIC Boost::system)

# Install targets and headers
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Export pluginlib information
pluginlib_export_plugin_description_file(hardware_interface
                                         perseus_lite_hardware.xml)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
