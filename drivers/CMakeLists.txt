add_library(bme68x_port STATIC
    bme68x_port.c
)

target_include_directories(bme68x_port PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(bme68x_port
    hardware_i2c
    pico_stdlib
    bme68x
)