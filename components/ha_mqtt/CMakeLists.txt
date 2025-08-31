idf_component_register(
  SRCS "ha_mqtt.c"
  INCLUDE_DIRS "include"
  REQUIRES mqtt esp_wifi esp_timer esp_netif
)
