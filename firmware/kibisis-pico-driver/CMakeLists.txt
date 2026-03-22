# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
  set(USERHOME $ENV{USERPROFILE})
else()
  set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if(EXISTS ${picoVscode})
  include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD
    pico2
    CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(kibisis_pico_driver C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Our modules
add_subdirectory(i2c_slave)
add_subdirectory(drive_motors)
add_subdirectory(quadrature_encoders)
add_subdirectory(space_resources_motor)
add_subdirectory(ldr_sensors)
add_subdirectory(status-light)

add_executable(kibisis_pico_driver kibisis_pico_driver.cpp)

pico_set_program_name(kibisis_pico_driver "kibisis_pico_driver")
pico_set_program_version(kibisis_pico_driver "0.1")

# Enable USB for debug output, disable UART
pico_enable_stdio_uart(kibisis_pico_driver 0)
pico_enable_stdio_usb(kibisis_pico_driver 1)

target_link_libraries(
  kibisis_pico_driver
  pico_stdlib
  hardware_i2c
  i2c_slave_lib
  drive_motors_lib
  quadrature_encoders_lib
  space_resources_motor_lib
  hardware_adc
  ldr_sensors_lib
  status_light_lib)

target_include_directories(
  kibisis_pico_driver PRIVATE ${CMAKE_CURRENT_LIST_DIR}
                              ${CMAKE_CURRENT_LIST_DIR}/ldr_sensors/include)

pico_add_extra_outputs(kibisis_pico_driver)
