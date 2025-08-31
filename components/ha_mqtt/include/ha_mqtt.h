#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *broker_uri;      // e.g. "mqtt://192.168.1.23"
    const char *username;        // NULL if none
    const char *password;        // NULL if none
    const char *friendly_name;   // e.g. "Living Room Presence"
    const char *suggested_area;  // e.g. "Living Room" (optional)
    const char *app_version;     // e.g. "0.2.0"
} ha_mqtt_cfg_t;

/** Initialize (does not connect yet). Safe to call once at boot. */
void ha_mqtt_init(const ha_mqtt_cfg_t *cfg);

/** Start MQTT client. Call after Wi-Fi is up (e.g. on IP_EVENT_STA_GOT_IP). */
void ha_mqtt_start(void);

/** Stop MQTT client (optional). */
void ha_mqtt_stop(void);

/** Publish presence + optional distance (mm). distance_mm < 0 if unknown. */
void ha_mqtt_publish_presence(bool present, int distance_mm);

/** Optionally publish RSSI immediately (otherwise it’s sent periodically). */
void ha_mqtt_publish_rssi_now(void);

/** Optionally force re-sending HA discovery configs (retained). */
void ha_mqtt_resend_discovery(void);

#ifdef __cplusplus
}
#endif
