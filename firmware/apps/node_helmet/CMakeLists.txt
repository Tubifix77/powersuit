# node_helmet_interface (Node 6) — ESP32-S3: mic/DAC audio bridge, ST7789 HUD,
# ESP-NOW diagnostics beacon.
cmake_minimum_required(VERSION 3.16)

set(SDKCONFIG_DEFAULTS "../../sdkconfig.defaults.common;sdkconfig.defaults")
set(EXTRA_COMPONENT_DIRS "../../components")

# Mirrors node_flight/CMakeLists.txt and firmware/components/ps_uros/CMakeLists.txt:
# the micro-ROS client is fetched on demand by firmware/tools/fetch_deps.sh and
# is gitignored, so a bench build without it must still configure cleanly
# (PS_UROS_ENABLED then compiles to 0 and the XRCE plane -- voice trigger,
# ambient temperature, HUD marker subscription -- is simply absent).
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../third_party/micro_ros_espidf_component")
    list(APPEND EXTRA_COMPONENT_DIRS "../../third_party/micro_ros_espidf_component")
endif()

# micro_ros_espidf_component (jazzy) reads ${PROJECT_DIR}/app-colcon.meta; the
# repo contract names the file colcon.meta. Mirror it so there is exactly one
# source of truth for the RMW sizing.
configure_file("${CMAKE_CURRENT_LIST_DIR}/colcon.meta"
               "${CMAKE_CURRENT_LIST_DIR}/app-colcon.meta" COPYONLY)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(node_helmet)
