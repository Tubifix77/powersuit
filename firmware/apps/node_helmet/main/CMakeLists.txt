idf_component_register(
    SRCS "app_main.c" "audio_in_task.c" "audio_out_task.c" "hud_task.c" "hud_model.c"
         "comms_task.c" "espnow_diag.c" "safety_glue.c"
    INCLUDE_DIRS "."
    REQUIRES ps_proto ps_can ps_safety ps_audio ps_uros
             esp_driver_i2s esp_driver_spi esp_driver_gpio esp_driver_tsens
             esp_lcd esp_wifi esp_netif nvs_flash esp_timer
)
