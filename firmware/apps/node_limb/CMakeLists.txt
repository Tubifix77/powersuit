# node_limb (Nodes 1-4) — ESP32-S3 limb actuation: 2-joint FOC + IMU + strain
# telemetry. One app image serves all four limbs; CONFIG_PS_LIMB_NODE_ID
# (Kconfig.projbuild) selects the board table and node identity at build time.
cmake_minimum_required(VERSION 3.16)

set(SDKCONFIG_DEFAULTS "../../sdkconfig.defaults.common;sdkconfig.defaults")
set(EXTRA_COMPONENT_DIRS "../../components")
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../third_party/micro_ros_espidf_component")
    list(APPEND EXTRA_COMPONENT_DIRS "../../third_party/micro_ros_espidf_component")
endif()

# micro_ros_espidf_component (jazzy) reads ${PROJECT_DIR}/app-colcon.meta; the
# repo contract names the file colcon.meta. Mirror it so there is exactly one
# source of truth for the RMW sizing.
configure_file("${CMAKE_CURRENT_LIST_DIR}/colcon.meta"
               "${CMAKE_CURRENT_LIST_DIR}/app-colcon.meta" COPYONLY)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(node_limb)
