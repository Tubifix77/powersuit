# Host-side build of the protocol contract library. Firmware builds the same
# sources through firmware/components/ps_proto (idf_component_register).
cmake_minimum_required(VERSION 3.16)
project(powersuit_proto C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(powersuit_proto STATIC
    src/crc16.c
    src/spi_frame.c
    src/adpcm_ima.c
)
target_include_directories(powersuit_proto PUBLIC include)
if(NOT MSVC)
    target_compile_options(powersuit_proto PRIVATE -Wall -Wextra -Werror)
endif()
