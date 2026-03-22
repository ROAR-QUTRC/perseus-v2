add_library(status_light_lib src/status-light.cpp)
target_include_directories(status_light_lib PUBLIC include)
target_link_libraries(status_light_lib pico_stdlib hardware_pio)

# Generate PIO header from ws2812.pio
pico_generate_pio_header(status_light_lib ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)
