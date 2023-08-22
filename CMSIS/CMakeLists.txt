cmake_minimum_required(VERSION 3.10.0)
project(cmsis)

option(ENABLE_RTOS "Use CMSIS RTOS" OFF)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/Device/Raspberry/RP2XXX)

add_library(${PROJECT_NAME} INTERFACE)
#target_link_libraries(${PROJECT_NAME} INTERFACE cmsis_rp20xx boot_stage2_asm)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR}/Core/Include)


#target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR}/Device/Raspberry/RP2XXX/Include)


# add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/RTOS2)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/DSP)
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/Device/Raspberry/RP2XXX)