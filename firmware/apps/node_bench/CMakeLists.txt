# Two-board bench bring-up image: real TWAI between two DevKitC-1s, with the
# onboard RGB LED reporting safety state. No motors, no soldering.
cmake_minimum_required(VERSION 3.16)
# Guarded so -D SDKCONFIG_DEFAULTS=... on the command line actually wins: a
# bare set() runs before project() and would clobber the cache value, which is
# how the orchestrator overlay selects the second role without menuconfig.
if(NOT SDKCONFIG_DEFAULTS)
    set(SDKCONFIG_DEFAULTS "../../sdkconfig.defaults.common;sdkconfig.defaults")
endif()
set(EXTRA_COMPONENT_DIRS "../../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(node_bench)
