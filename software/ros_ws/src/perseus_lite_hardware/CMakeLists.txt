cmake_minimum_required(VERSION 3.8)
project(perseus_lite_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) # Added Boost dependency

# Create the hardware interface library
add_library(${PROJECT_NAME} SHARED src/st3215_system.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  ${PROJECT_NAME} hardware_interface pluginlib rclcpp rclcpp_lifecycle
  Boost # Added Boost to target dependencies
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface
                                         perseus_lite_hardware.xml)

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle Boost # Added Boost to
                                                             # exported
                                                             # dependencies
)

ament_package()
