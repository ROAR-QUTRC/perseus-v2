cmake_minimum_required(VERSION 3.8)
project(perseus_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(THIS_PACKAGE_DEPS behaviortree_ros2 std_msgs btcpp_ros2_interfaces)

# ================================
# Add Execs
# ================================
set(EXEC1 perseus_bt_client)
add_executable(${EXEC1} src/${EXEC1}.cpp)
ament_target_dependencies(${EXEC1} ${THIS_PACKAGE_DEPS})

set(EXEC2 perseus_bt_exec)
add_executable(${EXEC2} src/${EXEC2}.cpp)
ament_target_dependencies(${EXEC2} ${THIS_PACKAGE_DEPS})

# ================================
# Install execs
# ================================
install(TARGETS ${EXEC1} ${EXEC2} DESTINATION lib/${PROJECT_NAME})

# ================================
# Install BT.xmls, ROS configs and launch files
# ================================
install(DIRECTORY behavior_trees config launch
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
