idf_component_register(
    SRCS "src/ps_spibridge.c"
    INCLUDE_DIRS "include"
    REQUIRES ps_proto driver esp_timer freertos
)
