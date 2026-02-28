cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(space_resources_end_effector C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(
  ${PROJECT_NAME}
  src/main.cpp
  src/protocol.cpp
  src/transceiver.cpp
  src/servo_driver.cpp
  src/current_sensor.cpp
  src/heater_control.cpp
  src/usb_repl.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE src)

target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_i2c hardware_adc
                      hardware_pwm)

# Enable USB output, disable UART output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Generate UF2 file for flashing
pico_add_extra_outputs(${PROJECT_NAME})
