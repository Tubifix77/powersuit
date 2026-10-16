# On-target self-test image for QEMU (esp32s3). Deliberately depends on no
# peripheral QEMU cannot emulate: no TWAI, MCPWM, I2S, SPI slave or ADC.
cmake_minimum_required(VERSION 3.16)
# Guarded so -D SDKCONFIG_DEFAULTS=... on the command line wins: a bare set()
# runs before project() and silently clobbers the cache value (node_bench).
if(NOT SDKCONFIG_DEFAULTS)
    set(SDKCONFIG_DEFAULTS "../../sdkconfig.defaults.common;sdkconfig.defaults")
endif()
set(EXTRA_COMPONENT_DIRS "../../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(node_qemu_test)
