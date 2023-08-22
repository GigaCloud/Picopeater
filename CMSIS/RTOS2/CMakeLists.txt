cmake_minimum_required(VERSION 3.10.0)
project(cmsis_rtos)

file(GLOB RTOS_SOURCE
    "${CMAKE_CURRENT_LIST_DIR}/Source/os_systick.c"
    "${CMAKE_CURRENT_LIST_DIR}/RTX/Source/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/RTX/Config/*.c")

if ((ARM_TOOLCHAIN_CORE STREQUAL "CORTEX_M0+") OR (ARM_TOOLCHAIN_CORE STREQUAL "CORTEX_M0"))
    set(CORE_FILE ${CMAKE_CURRENT_LIST_DIR}/RTX/Source/GCC/irq_armv6m.S)
else()
    message(FATAL_ERROR "Unsupported ARM toolchain core ${ARM_TOOLCHAIN_CORE}")
endif()

add_library(${PROJECT_NAME} STATIC ${RTOS_SOURCE} ${CORE_FILE})
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/Include
        ${CMAKE_CURRENT_LIST_DIR}/RTX/Include
        ${CMAKE_CURRENT_LIST_DIR}/RTX/Config
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/RTX/Source)

target_link_libraries(${PROJECT_NAME} PRIVATE cmsis)
target_compile_options(${PROJECT_NAME} PRIVATE "-fomit-frame-pointer")
target_compile_definitions(${PROJECT_NAME} PUBLIC "-DENABLE_RTOS")