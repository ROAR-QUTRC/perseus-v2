add_library(space_resources_motor_lib
    src/space_resources_motor.cpp
)

target_link_libraries(space_resources_motor_lib
    pico_stdlib
    hardware_pwm
)

target_include_directories(space_resources_motor_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)
