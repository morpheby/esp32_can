cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ESP_PLATFORM)
    if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        project(ESP32_CAN CXX)
    endif()
endif()

# Check if ch32_hal target is available
if(TARGET ch32_hal)
    set(CAN_CH32 ON)
else()
    set(CAN_CH32 OFF)
endif()

if(DEFINED ESP_PLATFORM)
    set(CAN_FREERTOS_ESP_STYLE ON)
else()
    set(CAN_FREERTOS_ESP_STYLE OFF)
endif()

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
    "${CMAKE_CURRENT_LIST_DIR}/src/freertos_inc.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/freertos_inc.h"
)

if (CAN_CH32)
    set(_CAN_SOURCES_BUILTIN "${_CAN_SOURCES_CH32}")
else()
    set(_CAN_SOURCES_BUILTIN "${_CAN_SOURCES_ESP32}")
endif()

set(CAN_RX_ON_MAIN_LOOP OFF CACHE BOOL "Use main loop instead of a separate task to conserve RAM")

if(DEFINED ESP_PLATFORM)
    idf_component_register(
        SRCS
            ${_CAN_SOURCES}
            ${_CAN_SOURCES_BUILTIN}
        INCLUDE_DIRS
            "${CMAKE_CURRENT_SOURCE_DIR}/src"
            "${CMAKE_CURRENT_BINARY_DIR}"
        REQUIRES
            arduino-esp32
            can_common
    )
else()
    add_library(esp32_can STATIC
        ${_CAN_SOURCES}
        ${_CAN_SOURCES_BUILTIN}
    )
    target_include_directories(esp32_can PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_BINARY_DIR}")
    target_link_libraries(esp32_can PUBLIC can_common)
endif()
