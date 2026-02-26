cmake_minimum_required(VERSION 3.10)
project(perseus_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf ros2_control rviz meshes launch
        DESTINATION share/${PROJECT_NAME})
ament_package()
