idf_component_register(
    SRCS "src/ps_can.c"
    INCLUDE_DIRS "include"
    REQUIRES ps_proto esp_driver_twai
    PRIV_REQUIRES esp_timer
)
