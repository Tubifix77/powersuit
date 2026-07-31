# Host tests for every pure-C module. Runs anywhere with gcc/clang + CMake —
# in practice inside the espressif/idf image (has both) via firmware/tools/.
# The explicit test list is part of the contract: builds fail until every
# workstream has delivered its pure sources and tests.
cmake_minimum_required(VERSION 3.16)
project(powersuit_host_tests C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
enable_testing()

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(FW "${REPO_ROOT}/firmware")

add_subdirectory("${REPO_ROOT}/common/c" common_c)

# Pure sources contributed by the firmware components (no IDF dependencies).
add_library(ps_pure STATIC
    "${FW}/components/ps_ctl/src/foc_math.c"
    "${FW}/components/ps_ctl/src/pid.c"
    "${FW}/components/ps_ctl/src/imu_mahony.c"
    "${FW}/components/ps_safety/src/ps_safety_core.c"
    "${FW}/components/ps_router/src/ps_router.c"
    "${FW}/components/ps_audio/src/audio_pkt.c"
    "${FW}/components/ps_audio/src/energy_gate.c"
)
target_include_directories(ps_pure PUBLIC
    "${FW}/components/ps_ctl/include"
    "${FW}/components/ps_safety/include"
    "${FW}/components/ps_router/include"
    "${FW}/components/ps_audio/include"
)
target_link_libraries(ps_pure PUBLIC powersuit_proto)
target_compile_options(ps_pure PRIVATE -Wall -Wextra -Werror)

set(PS_HOST_TESTS
    test_crc16
    test_can_id
    test_spi_frame
    test_adpcm
    test_foc_math
    test_pid
    test_imu_filter
    test_safety_core
    test_router
    test_audio_pkt
    test_energy_gate
)

foreach(t IN LISTS PS_HOST_TESTS)
    add_executable(${t} ${t}.c)
    target_link_libraries(${t} PRIVATE ps_pure m)
    target_include_directories(${t} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_compile_options(${t} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${t} COMMAND ${t})
endforeach()
