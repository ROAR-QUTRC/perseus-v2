cmake_minimum_required(VERSION 3.5)
project(perseus_lite_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf ros2_control rviz meshes launch
        DESTINATION share/${PROJECT_NAME})
ament_package()
