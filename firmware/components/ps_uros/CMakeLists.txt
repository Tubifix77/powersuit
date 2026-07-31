# micro-ROS is an optional heavyweight dependency: the vendored client is cloned
# on demand by firmware/tools/fetch_deps.sh. A limb node must still build and
# flash for bench bring-up without it, so the component is detected here and the
# XRCE plane is compiled out when it is absent (see src/ps_uros_task.c).
get_filename_component(_ps_uros_dir
    "${CMAKE_CURRENT_LIST_DIR}/../../third_party/micro_ros_espidf_component" ABSOLUTE)

if(EXISTS "${_ps_uros_dir}/CMakeLists.txt")
    set(_ps_uros_req micro_ros_espidf_component)
    set(_ps_uros_enabled 1)
else()
    set(_ps_uros_req "")
    set(_ps_uros_enabled 0)
    message(STATUS "ps_uros: micro-ROS client not vendored; XRCE plane disabled "
                   "(run firmware/tools/fetch_deps.sh to enable)")
endif()

idf_component_register(
    SRCS "src/ps_uros_transport_can.c" "src/ps_uros_task.c"
    INCLUDE_DIRS "include"
    REQUIRES ps_proto ps_can ${_ps_uros_req}
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC PS_UROS_ENABLED=${_ps_uros_enabled})
