cmake_minimum_required(VERSION 3.8)
project(perseus_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# Dependency for the interface generation
find_package(rosidl_default_generators REQUIRED)

# Add action files
rosidl_generate_interfaces(${PROJECT_NAME} "action/BTStart.action" DEPENDENCIES
                           builtin_interfaces action_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
