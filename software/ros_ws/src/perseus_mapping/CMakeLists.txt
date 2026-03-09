cmake_minimum_required(VERSION 3.10)
project(perseus_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config
            launch
            maps
            rviz
            urdf
            worlds
            src
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sim_pointcloud_filter src/sim_pointcloud_filter.cpp)
ament_target_dependencies(sim_pointcloud_filter rclcpp sensor_msgs)

install(TARGETS sim_pointcloud_filter DESTINATION lib/${PROJECT_NAME})

install(TARGETS sim_pointcloud_filter DESTINATION lib/${PROJECT_NAME})

ament_package()
