cmake_minimum_required(VERSION 3.5)
project(perseus_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf ros2_control rviz meshes
        DESTINATION share/${PROJECT_NAME})
ament_package()
