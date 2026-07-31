idf_component_register(
    SRCS "src/ps_safety_core.c" "src/ps_safety_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES ps_proto ps_can
    PRIV_REQUIRES esp_timer
)
