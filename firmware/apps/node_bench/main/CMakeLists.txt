idf_component_register(
    SRCS "bench_main.c" "can_selftest.c"
    INCLUDE_DIRS "."
    REQUIRES ps_proto ps_can ps_safety driver esp_timer
)
