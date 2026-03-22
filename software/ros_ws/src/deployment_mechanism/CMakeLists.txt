cmake_minimum_required(VERSION 3.8)
project(deployment_mechanism)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIODCXX REQUIRED libgpiodcxx)

add_executable(deployment_mechanism src/deployment_mechanism.cpp)

target_include_directories(deployment_mechanism
                           PRIVATE include ${GPIODCXX_INCLUDE_DIRS})

target_link_libraries(deployment_mechanism ${GPIODCXX_LIBRARIES})

ament_target_dependencies(deployment_mechanism rclcpp std_msgs)

install(TARGETS deployment_mechanism DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
