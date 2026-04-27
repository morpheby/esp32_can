cmake_minimum_required(VERSION 3.16)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    project(ESP32_CAN CXX)
endif()

# Check if ch32_hal target is available
if(TARGET ch32_hal)
    set(CAN_CH32 ON)
else()
    set(CAN_CH32 OFF)
endif()
check_include_file("freertos/FreeRTOS.h" CAN_FREERTOS_ESP_STYLE)

set(_CAN_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mcp2517fd.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mcp2515.cpp"
)

set(_CAN_SOURCES_CH32
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ch32v203_can_builtin.cpp"
)

set(_CAN_SOURCES_ESP32
    "${CMAKE_CURRENT_SOURCE_DIR}/src/esp32_can_builtin.cpp"
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/freertos_inc.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/freertos_inc.h"
)

if (CAN_CH32)
    set(_CAN_SOURCES_BUILTIN "${_CAN_SOURCES_CH32}")
else()
    set(_CAN_SOURCES_BUILTIN "${_CAN_SOURCES_ESP32}")
endif()

add_library(esp32_can STATIC
    ${_CAN_SOURCES}
    ${_CAN_SOURCES_BUILTIN}
)

set(CAN_RX_ON_MAIN_LOOP OFF CACHE BOOL "Use main loop instead of a separate task to conserve RAM")

target_include_directories(esp32_can PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_BINARY_DIR}")

if(CAN_RX_ON_MAIN_LOOP)
    target_compile_definitions(esp32_can PUBLIC CAN_RX_ON_MAIN_LOOP)
endif()

target_link_libraries(esp32_can PUBLIC can_common)
