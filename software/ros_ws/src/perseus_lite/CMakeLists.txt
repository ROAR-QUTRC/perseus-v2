# PROJECT SETUP
cmake_minimum_required(VERSION 3.16)

project(perseus_lite VERSION 0.0.1)

find_package(ament_cmake REQUIRED)

INSTALL install(DIRECTORY description/urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY bringup/launch bringup/config
        DESTINATION share/${PROJECT_NAME})

ament_package()
