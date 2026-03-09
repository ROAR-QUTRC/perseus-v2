add_library(moisture_sensor_lib
    src/moisture_sensor.cpp
)

target_include_directories(moisture_sensor_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(moisture_sensor_lib
    pico_stdlib
    hardware_adc
)
