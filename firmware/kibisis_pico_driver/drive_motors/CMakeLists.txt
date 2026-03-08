add_library(drive_motors_lib
    src/drive_motors.cpp
)

target_link_libraries(drive_motors_lib
    pico_stdlib
    hardware_pwm
    hardware_gpio
)

target_include_directories(drive_motors_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)
