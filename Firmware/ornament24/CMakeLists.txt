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
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
set(PICO_EXTRAS_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-extras)
include(pico_extras_import.cmake)


project(ornament24 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ornament24 
ornament24.c
lis3dh.c
)

add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)

pico_set_program_name(ornament24 "ornament24")
pico_set_program_version(ornament24 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ornament24 0)
pico_enable_stdio_usb(ornament24 1)



# Add the standard include files to the build
target_include_directories(ornament24 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)
# Add the standard library to the build
target_link_libraries(ornament24
        pico_stdlib
        hardware_pwm
        hardware_i2c
        hardware_adc
        hardware_sync
        hardware_sleep
        hardware_watchdog
        hardware_clocks
        hardware_rtc
        )

pico_add_extra_outputs(ornament24)

