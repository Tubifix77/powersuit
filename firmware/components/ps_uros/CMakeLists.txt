# micro-ROS is an optional heavyweight dependency: the vendored client is cloned
# on demand by firmware/tools/fetch_deps.sh. A limb node must still build and
# flash for bench bring-up without it, so the component is detected here and the
# XRCE plane is compiled out when it is absent (see src/ps_uros_task.c).
# Detect whether the client is in THIS build, not merely present on disk. Those
# differ: node_chest_hub and node_qemu_test deliberately omit third_party from
# EXTRA_COMPONENT_DIRS, and a directory-existence check would make them require
# a component that was never added — breaking them the moment anyone runs
# fetch_deps.sh for a different app.
idf_build_get_property(_ps_uros_dirs BUILD_COMPONENT_DIRS)
set(_ps_uros_enabled 0)
foreach(_d IN LISTS _ps_uros_dirs)
    if(_d MATCHES "micro_ros_espidf_component")
        set(_ps_uros_enabled 1)
    endif()
endforeach()

if(_ps_uros_enabled)
    set(_ps_uros_req micro_ros_espidf_component)
else()
    set(_ps_uros_req "")
    message(STATUS "ps_uros: micro-ROS client not in this build; XRCE plane disabled "
                   "(vendor it with firmware/tools/fetch_deps.sh and add it to "
                   "the app's EXTRA_COMPONENT_DIRS to enable)")
endif()

idf_component_register(
    SRCS "src/ps_uros_transport_can.c" "src/ps_uros_task.c"
    INCLUDE_DIRS "include"
    REQUIRES ps_proto ps_can ${_ps_uros_req}
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC PS_UROS_ENABLED=${_ps_uros_enabled})
