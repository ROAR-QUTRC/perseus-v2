# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(kibisis_pico_driver C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# ── pico_i2c_slave ────────────────────────────────────────────────────────────
#
# The vmilea/pico_i2c_slave library was upstreamed into the Pico SDK as of
# SDK 1.5.0. SDK 2.2.0 is in use here so it is already bundled – just link it.
#
# If you ever need to pin to the vmilea repo instead, uncomment this block:
#
# include(FetchContent)
# FetchContent_Declare(
#     pico_i2c_slave
#     GIT_REPOSITORY https://github.com/vmilea/pico_i2c_slave.git
#     GIT_TAG        master
# )
# FetchContent_MakeAvailable(pico_i2c_slave)
# add_subdirectory(${pico_i2c_slave_SOURCE_DIR}/i2c_slave i2c_slave)
#
# ─────────────────────────────────────────────────────────────────────────────

add_executable(kibisis_pico_driver
    kibisis_pico_driver.cpp
    src/kibisis_pwm_converter.cpp
    src/kibisis_encoder.cpp
)

pico_set_program_name(kibisis_pico_driver "kibisis_pico_driver")
pico_set_program_version(kibisis_pico_driver "0.1")

pico_enable_stdio_uart(kibisis_pico_driver 0)
pico_enable_stdio_usb(kibisis_pico_driver 1)

# Generate the quadrature_encoder.pio.h header from the .pio source
pico_generate_pio_header(kibisis_pico_driver
    ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio
)

# Add the standard library to the build
target_link_libraries(kibisis_pico_driver
    pico_stdlib
)

target_include_directories(kibisis_pico_driver PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# Add any user requested libraries
target_link_libraries(kibisis_pico_driver
    hardware_i2c
    hardware_pwm
    hardware_gpio
    hardware_watchdog
    hardware_pio               # PIO for quadrature encoder
    hardware_adc
    pico_i2c_slave
)

pico_add_extra_outputs(kibisis_pico_driver)
