add_library(quadrature_encoders_lib
    src/quadrature_encoders.cpp
)

# Generate the .pio.h header from the .pio source file
pico_generate_pio_header(quadrature_encoders_lib
    ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio
)

target_include_directories(quadrature_encoders_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(quadrature_encoders_lib
    pico_stdlib
    hardware_pio
    hardware_clocks
)
