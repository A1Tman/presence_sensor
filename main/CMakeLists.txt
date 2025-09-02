idf_component_register(
  SRCS "main.c"
  INCLUDE_DIRS "."
  PRIV_REQUIRES
      ha_mqtt
      ld2420_simple
      ld2420
      ld2411
      esp_wifi
      esp_timer
      esp_netif
      nvs_flash        
      spi_flash        
)
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_SOURCE_DIR}/config")
