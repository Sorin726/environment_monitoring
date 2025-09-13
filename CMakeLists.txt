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
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(environment_monitoring C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(environment_monitoring main.c )

pico_set_program_name(environment_monitoring "environment_monitoring")
pico_set_program_version(environment_monitoring "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(environment_monitoring 1)
pico_enable_stdio_usb(environment_monitoring 0)

# Add the standard library to the build
target_link_libraries(environment_monitoring
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(environment_monitoring PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add BME68X library
add_subdirectory(lib/bme68x_cmake)
add_subdirectory(drivers)

# Add BSEC library
# set(BSEC_PATH "C:/Users/Sorin/OneDrive - Universitatea Politehnica Bucuresti/Documents/bsec2-6-1-0_generic_release_22102024")
add_library(bsec STATIC IMPORTED)
set_target_properties(bsec PROPERTIES
    IMPORTED_LOCATION $ENV{BSEC_PATH}/algo/bsec_IAQ/bin/gcc/Cortex_M33/libalgobsec.a
    INTERFACE_INCLUDE_DIRECTORIES $ENV{BSEC_PATH}/algo/bsec_IAQ/inc/
)

set(CMAKE_BUILD_TYPE Debug)


# Add any user requested libraries
target_link_libraries(environment_monitoring 
        bsec
        bme68x_port
        bme68x
        hardware_i2c
        pico_cyw43_arch_none
        )

pico_add_extra_outputs(environment_monitoring)