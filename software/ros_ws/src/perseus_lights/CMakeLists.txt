cmake_minimum_required(VERSION 3.8...3.28)
project(perseus_lights)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp std_msgs sensor_msgs nav_msgs)
set(THIS_PACKAGE_BUILD_DEPENDS ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(hi_can_raw QUIET)
if(NOT hi_can_raw_FOUND)
  message(STATUS "hi-can-raw not found, adding subdirectory...")
  message(
    STATUS "Attempting to add hi-can-raw from: ../../../shared/hi-can-raw")
  add_subdirectory(../../../shared/hi-can-raw hi-can-raw)
endif()

# Find execs
file(GLOB_RECURSE LIGHT_DRIVER_CODE_SOURCES src/light_driver.cpp)
file(GLOB_RECURSE LIGHT_CONTROLLER_CODE_SOURCES src/light_controller.cpp)
file(GLOB_RECURSE LIGHT_ORCHESTRATOR_CODE_SOURCES src/light_orchestrator.cpp)

# Add exec targets
add_executable(light_driver ${LIGHT_DRIVER_CODE_SOURCES})
add_executable(light_controller ${LIGHT_CONTROLLER_CODE_SOURCES})
add_executable(light_orchestrator ${LIGHT_ORCHESTRATOR_CODE_SOURCES})

# Add include dirs for hpp files
target_include_directories(light_driver
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(light_controller
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(light_orchestrator
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add dependencies for exec
ament_target_dependencies(light_driver PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
                          std_msgs)
ament_target_dependencies(light_controller PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS} std_msgs)
ament_target_dependencies(
  light_orchestrator PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS} std_msgs
  sensor_msgs nav_msgs)

target_link_libraries(light_driver PUBLIC hi_can_raw)
target_link_libraries(light_controller PUBLIC hi_can_raw)
target_link_libraries(light_orchestrator PUBLIC hi_can_raw)

# Install ROS things
install(TARGETS light_driver DESTINATION lib/${PROJECT_NAME})
install(TARGETS light_controller DESTINATION lib/${PROJECT_NAME})
install(TARGETS light_orchestrator DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
