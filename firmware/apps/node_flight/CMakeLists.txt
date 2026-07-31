# node_flight_actuation (Node 5) — ESP32-S3 flap servos + aero engine.
cmake_minimum_required(VERSION 3.16)

set(SDKCONFIG_DEFAULTS "../../sdkconfig.defaults.common;sdkconfig.defaults")
set(EXTRA_COMPONENT_DIRS
    "../../components"
    "../../third_party/micro_ros_espidf_component"
)

# micro_ros_espidf_component (jazzy) reads ${PROJECT_DIR}/app-colcon.meta; the
# repo contract names the file colcon.meta. Mirror it so there is exactly one
# source of truth for the RMW sizing.
configure_file("${CMAKE_CURRENT_LIST_DIR}/colcon.meta"
               "${CMAKE_CURRENT_LIST_DIR}/app-colcon.meta" COPYONLY)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(node_flight)
