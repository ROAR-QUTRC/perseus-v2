cmake_minimum_required(VERSION 3.10)
project(perseus_mapping)

# Launch and config only: this package builds no targets, it just installs the FAST-LIO
# parameters and the launch file that feeds them in.
find_package(ament_cmake REQUIRED)

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
  DIRECTORY config launch maps rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
