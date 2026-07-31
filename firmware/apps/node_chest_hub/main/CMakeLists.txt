idf_component_register(
    SRCS "app_main.c" "gateway_task.c" "bms_task.c" "led_task.c"
    INCLUDE_DIRS "."
    REQUIRES ps_proto ps_can ps_safety ps_router ps_spibridge driver esp_adc esp_timer
)
