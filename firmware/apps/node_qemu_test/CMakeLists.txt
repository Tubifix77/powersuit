# On-target self-test image for QEMU (esp32s3). Deliberately depends on no
# peripheral QEMU cannot emulate: no TWAI, MCPWM, I2S, SPI slave or ADC.
cmake_minimum_required(VERSION 3.16)
set(SDKCONFIG_DEFAULTS "../../sdkconfig.defaults.common;sdkconfig.defaults")
set(EXTRA_COMPONENT_DIRS "../../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(node_qemu_test)
