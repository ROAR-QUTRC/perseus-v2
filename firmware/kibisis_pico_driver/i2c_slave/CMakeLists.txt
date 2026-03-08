add_library(i2c_slave_lib
    src/i2c_slave.cpp
)

target_link_libraries(i2c_slave_lib
    pico_stdlib
    hardware_i2c
)

# PUBLIC so that any target linking i2c_slave_lib automatically gets this include path
target_include_directories(i2c_slave_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)
