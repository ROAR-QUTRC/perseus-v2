cmake_minimum_required(VERSION 3.10)
project(perseus-arm-teleop)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Boost REQUIRED COMPONENTS system)
find_package(Curses REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add include directory
include_directories(${PROJECT_SOURCE_DIR}/include ${CURSES_INCLUDE_DIR})

# Add executable
add_executable(${PROJECT_NAME} 
    main.cpp
    src/perseus-arm-teleop.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Boost::system
    ${CURSES_LIBRARIES}
    yaml-cpp
    pthread
)