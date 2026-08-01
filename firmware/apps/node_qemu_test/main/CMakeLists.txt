idf_component_register(
    SRCS "qemu_selftest.c"
    INCLUDE_DIRS "." "../../../tests/host"
    REQUIRES ps_proto ps_safety ps_ctl ps_router esp_timer
)
