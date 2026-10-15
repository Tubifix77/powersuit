idf_component_register(
    SRCS "src/ps_can.c" "src/ps_can_txq.c"
    INCLUDE_DIRS "include"
    REQUIRES ps_proto esp_driver_twai
    PRIV_REQUIRES esp_timer
)
