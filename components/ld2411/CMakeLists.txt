idf_component_register(
    SRCS "ld2411.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer nvs_flash sensor_info
    PRIV_REQUIRES ha_mqtt
)
