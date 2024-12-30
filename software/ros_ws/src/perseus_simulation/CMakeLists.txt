cmake_minimum_required(VERSION 3.8)
project(perseus_simulation)

find_package(ament_cmake REQUIRED)

# Install only existing directories
foreach(dir launch worlds config)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
