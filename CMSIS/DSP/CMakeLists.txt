cmake_minimum_required(VERSION 3.10.0)
project(cmsis_dsp)

file(GLOB_RECURSE SOURCE
    Source/*.c)

add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/Include
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/PrivateInclude)
target_link_libraries(${PROJECT_NAME} PRIVATE cmsis)