cmake_minimum_required(VERSION 3.13r)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(tesis_sonar C CXX ASM)
#project(tesis_pub_only C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(PICO_BOARD pico_w)

pico_sdk_init()

#link_directories($ENV{LIB_MICROROS_PATH}/libmicroros)

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.c
    pico_uart_transport.c
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    PICO_DEFAULT_UART_BAUD_RATE=921600
)
target_link_libraries(${PROJECT_NAME}
    hardware_i2c
    pico_stdlib
    #microros
    hardware_timer
    # Enlace directo al archivo físico utilizando la variable de entorno
    $ENV{LIB_MICROROS_PATH}/libmicroros.a
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $ENV{LIB_MICROROS_PATH}/libmicroros/include
    $ENV{LIB_MICROROS_PATH}/include
    $ENV{PICO_SDK_PATH}
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico (Activar UART / Desactivar Micro USB)
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(${PROJECT_NAME})

message(STATUS "PICO SDK PATH: $ENV{PICO_SDK_PATH}")
message(STATUS "LIB MICROROS PATH: $ENV{LIB_MICROROS_PATH}")
