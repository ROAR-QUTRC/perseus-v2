# PROJECT SETUP
cmake_minimum_required(VERSION 3.16)

project(perseus VERSION 0.0.1)

find_package(ament_cmake REQUIRED)

# INSTALL
install(DIRECTORY description/urdf DESTINATION share/perseus)
install(DIRECTORY bringup/config DESTINATION share/perseus)

ament_package()
