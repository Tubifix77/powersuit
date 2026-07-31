idf_component_register(
    SRCS "app_main.c" "control_task.c" "comms_task.c" "safety_glue.c" "imu_mpu6050.c"
    INCLUDE_DIRS "."
    REQUIRES ps_proto ps_can ps_safety ps_ctl ps_uros driver esp_adc esp_timer
              esp_driver_gptimer esp_driver_spi esp_driver_i2c
)
