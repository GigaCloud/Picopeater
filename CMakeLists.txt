# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/radu/Desktop/Picopeater/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(Picopeater C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Picopeater Picopeater.c )

pico_set_program_name(Picopeater "Picopeater")
pico_set_program_version(Picopeater "0.1")

pico_enable_stdio_uart(Picopeater 1)
pico_enable_stdio_usb(Picopeater 0)


target_compile_definitions(Picopeater PRIVATE
  PICO_DEFAULT_UART_TX_PIN=8
  PICO_DEFAULT_UART_RX_PIN=9
  PICO_DEFAULT_UART_INSTANCE=uart1
)

# Add the standard library to the build
target_link_libraries(Picopeater
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Picopeater PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(Picopeater
        hardware_pio
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_adc
        )

pico_add_extra_outputs(Picopeater)

