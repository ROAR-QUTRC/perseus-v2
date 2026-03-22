add_library(i2c_slave_lib src/i2c_slave.cpp)
target_include_directories(i2c_slave_lib PUBLIC include)
target_link_libraries(i2c_slave_lib pico_stdlib hardware_i2c hardware_irq
                      hardware_sync)
