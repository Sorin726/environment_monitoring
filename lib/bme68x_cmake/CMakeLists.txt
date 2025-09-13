add_library(bme68x STATIC
    ../bme68x/bme68x.c
)

target_include_directories(bme68x PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../bme68x
)
