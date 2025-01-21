cmake_minimum_required(VERSION 3.8)
project(perseus_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_common REQUIRED)

set(THIS_PACKAGE_DEPS behaviortree_ros2 std_msgs btcpp_ros2_interfaces)

# ==================================
# Install BT.xmls, launch and params
# ==================================
install(DIRECTORY behavior_trees launch params
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
