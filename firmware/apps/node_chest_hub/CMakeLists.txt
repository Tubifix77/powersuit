# Node 7 — node_chest_power_hub (ESP32-P4): dual-CAN gateway, SPI bridge, BMS.
# Deliberately NO micro-ROS component dirs here: the hub runs no on-chip
# micro-ROS client (docs/network-map.md §12.4); it speaks raw TELEM/MGMT via
# SPI records and the Node 8 bridge publishes its topics.
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "../../components")
set(SDKCONFIG_DEFAULTS "../../sdkconfig.defaults.common;sdkconfig.defaults")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(node_chest_hub)
