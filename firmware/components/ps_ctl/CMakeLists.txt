# Pure math (foc_math/pid/imu_mahony) is shared with the host test build in
# firmware/tests/host; foc_drv_mcpwm is the ESP32 output stage. Encoder and ADC
# reading live in each app's board layer because the wiring is board-specific.
idf_component_register(
    SRCS "src/foc_math.c" "src/pid.c" "src/imu_mahony.c" "src/foc_drv_mcpwm.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_mcpwm
    PRIV_REQUIRES esp_timer
)
