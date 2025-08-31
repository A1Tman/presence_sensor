#include "ha_mqtt.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mqtt_client.h"

/* ======================= Build-time knobs ======================= */
#define HA_DISC_PREFIX                      "homeassistant"
#define DISTANCE_ENABLED_BY_DEFAULT         1   /* set to 1 once UART distance publishes */
#define MIN_DIAG_INTERVAL_SEC               60  /* uptime & RSSI cadence */

/* ======================= Module state ======================= */
static const char *TAG = "ha_mqtt";

static ha_mqtt_cfg_t s_cfg = {
    .broker_uri    = "mqtt://192.168.1.23",
    .username      = NULL,
    .password      = NULL,
    .friendly_name = "LD2420 Presence",
    .suggested_area= NULL,
    .app_version   = "0.2.0"
};

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;

/* Derived identifiers & topics */
static char s_mac_str[18];              // AA:BB:CC:DD:EE:FF
static char s_devid[32];                // ld2420c3-aabbcc
static char s_topic_base[64];           // ld2420_c3/ld2420c3-aabbcc
static char s_topic_status[96];
static char s_topic_presence[96];
static char s_topic_distance_cm[96];
static char s_topic_attrs[96];
static char s_topic_rssi[96];
static char s_topic_uptime[96];

/* HA Discovery topics */
static char s_disc_bs_presence[128];
static char s_disc_sensor_distance[128];
static char s_disc_sensor_rssi[128];
static char s_disc_sensor_uptime[128];

/* Uptime ticker */
static int64_t s_boot_us = 0;
static int64_t s_last_diag_us = 0;  // publish diagnostics every ~60s

/* ======================= Utilities ======================= */
static void mac_to_str(uint8_t mac[6], char out[18]) {
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void derive_ids_and_topics(void) {
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    mac_to_str(mac, s_mac_str);

    /* short hex id (last 3 bytes) */
    snprintf(s_devid, sizeof(s_devid), "ld2420c3-%02x%02x%02x", mac[3], mac[4], mac[5]);

    snprintf(s_topic_base, sizeof(s_topic_base), "ld2420_c3/%s", s_devid);
    snprintf(s_topic_status, sizeof(s_topic_status),    "%s/status",      s_topic_base);
    snprintf(s_topic_presence, sizeof(s_topic_presence),"%s/presence",    s_topic_base);
    snprintf(s_topic_distance_cm, sizeof(s_topic_distance_cm), "%s/distance_cm", s_topic_base);
    snprintf(s_topic_attrs, sizeof(s_topic_attrs),      "%s/attributes",  s_topic_base);
    snprintf(s_topic_rssi, sizeof(s_topic_rssi),        "%s/rssi",        s_topic_base);
    snprintf(s_topic_uptime, sizeof(s_topic_uptime),    "%s/uptime_s",    s_topic_base);

    /* Home Assistant discovery topics (retain) */
    snprintf(s_disc_bs_presence, sizeof(s_disc_bs_presence),
             HA_DISC_PREFIX "/binary_sensor/%s/presence/config", s_devid);
    snprintf(s_disc_sensor_distance, sizeof(s_disc_sensor_distance),
             HA_DISC_PREFIX "/sensor/%s/distance/config", s_devid);
    snprintf(s_disc_sensor_rssi, sizeof(s_disc_sensor_rssi),
             HA_DISC_PREFIX "/sensor/%s/rssi/config", s_devid);
    snprintf(s_disc_sensor_uptime, sizeof(s_disc_sensor_uptime),
             HA_DISC_PREFIX "/sensor/%s/uptime/config", s_devid);
}

static void pub(const char *topic, const char *payload, int qos, int retain) {
    if (!s_client) return;
    int mid = esp_mqtt_client_publish(s_client, topic, payload, 0, qos, retain);
    if (mid < 0) ESP_LOGW(TAG, "Publish failed to %s", topic);
}

/* ======================= Discovery payloads ======================= */
static void publish_discovery_all(void) {
    char dev_block[512];
    char area[96] = {0};
    const char *dev_name = s_cfg.friendly_name ? s_cfg.friendly_name : "LD2420 Presence";

    if (s_cfg.suggested_area && s_cfg.suggested_area[0]) {
        snprintf(area, sizeof(area), ",\"suggested_area\":\"%s\"", s_cfg.suggested_area);
    }

    /* Device object (only here; entities keep generic names) */
    snprintf(dev_block, sizeof(dev_block),
        "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"Hi-Link + DIY\","
        "\"mdl\":\"HLK-LD2420 + ESP32-C3\",\"sw\":\"%s\","
        "\"connections\":[[\"mac\",\"%s\"]]}%s",
        s_devid, dev_name,
        s_cfg.app_version ? s_cfg.app_version : "0.0.0",
        s_mac_str,
        area
    );

    /* Presence (binary_sensor) — clean name */
    {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Presence\","
            "\"uniq_id\":\"%s_presence\","
            "\"stat_t\":\"%s\","
            "\"dev_cla\":\"occupancy\","
            "\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"json_attr_t\":\"%s\","
            "\"obj_id\":\"presence\",",
            s_devid, s_topic_presence,
            s_topic_status,
            s_topic_attrs
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_bs_presence, payload, 1, 1);
    }

    /* Distance (sensor) — cm, disabled by default until UART feed ready */
    {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Distance\","
            "\"uniq_id\":\"%s_distance\","
            "\"stat_t\":\"%s\","
            "\"dev_cla\":\"distance\","
            "\"unit_of_meas\":\"cm\",\"stat_cla\":\"measurement\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"obj_id\":\"distance\","
            "\"enabled_by_default\":%s,",
            s_devid, s_topic_distance_cm,
            s_topic_status,
            DISTANCE_ENABLED_BY_DEFAULT ? "true" : "false"
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_sensor_distance, payload, 1, 1);
    }

    /* RSSI (sensor) — dBm, diagnostic */
    {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"RSSI\","
            "\"uniq_id\":\"%s_rssi\","
            "\"stat_t\":\"%s\","
            "\"dev_cla\":\"signal_strength\",\"unit_of_meas\":\"dBm\",\"ent_cat\":\"diagnostic\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"obj_id\":\"rssi\",",
            s_devid, s_topic_rssi,
            s_topic_status
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_sensor_rssi, payload, 1, 1);
    }

    /* Uptime (sensor) — seconds, diagnostic */
    {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Uptime\","
            "\"uniq_id\":\"%s_uptime\","
            "\"stat_t\":\"%s\","
            "\"unit_of_meas\":\"s\",\"stat_cla\":\"measurement\",\"ent_cat\":\"diagnostic\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"obj_id\":\"uptime\",",
            s_devid, s_topic_uptime,
            s_topic_status
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_sensor_uptime, payload, 1, 1);
    }
}

/* Availability birth message */
static void publish_birth_online(void) {
    pub(s_topic_status, "online", 1, 1);
}

/* Periodic diagnostics: uptime + RSSI */
static void publish_periodic_diag_if_due(void) {
    const int64_t now_us = esp_timer_get_time();
    if (now_us - s_last_diag_us < (int64_t)MIN_DIAG_INTERVAL_SEC * 1000000LL) return;
    s_last_diag_us = now_us;

    int64_t uptime_s = (now_us - s_boot_us) / 1000000LL;
    char buf[32];
    snprintf(buf, sizeof(buf), "%" PRId64, uptime_s);
    pub(s_topic_uptime, buf, 0, 0);

    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        char rssi[16];
        snprintf(rssi, sizeof(rssi), "%d", ap.rssi);
        pub(s_topic_rssi, rssi, 0, 0);
    }

    /* Keep HA availability fresh (retained) */
    pub(s_topic_status, "online", 1, 1);
}

/* Attributes blob (mac, fw, ip, etc.) */
static void publish_attrs_once(void) {
    esp_netif_ip_info_t ip;
    char json[512];
    char ip_str[32] = "0.0.0.0";

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip.ip));
    }
    snprintf(json, sizeof(json),
        "{"
        "\"mac\":\"%s\","
        "\"device_id\":\"%s\","
        "\"model\":\"HLK-LD2420 + ESP32-C3\","
        "\"sw_version\":\"%s\","
        "\"ip\":\"%s\""
        "}",
        s_mac_str, s_devid,
        s_cfg.app_version ? s_cfg.app_version : "0.0.0",
        ip_str
    );
    pub(s_topic_attrs, json, 0, 0);
}

/* ======================= MQTT event handling ======================= */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            s_connected = true;
            ESP_LOGI(TAG, "MQTT connected (session_present=%d)", e ? e->session_present : -1);
            publish_discovery_all();
            publish_birth_online();
            publish_attrs_once();
            publish_periodic_diag_if_due();
            break;

        case MQTT_EVENT_DISCONNECTED:
            s_connected = false;
            ESP_LOGW(TAG, "MQTT disconnected");
            break;

        case MQTT_EVENT_ERROR:
            if (e && e->error_handle) {
                const esp_mqtt_error_codes_t *err = e->error_handle;
                ESP_LOGW(TAG,
                         "MQTT error: type=%d, esp_err=0x%x, tls_stack=0x%x, conn_refused=%d",
                         err->error_type,
                         err->esp_tls_last_esp_err,
                         err->esp_tls_stack_err,
                         err->connect_return_code);
            } else {
                ESP_LOGW(TAG, "MQTT error");
            }
            break;

        case MQTT_EVENT_DATA:
#if LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG
            if (e && e->topic && e->data)
                ESP_LOGD(TAG, "MQTT data: topic='%.*s' payload='%.*s'",
                         e->topic_len, e->topic, e->data_len, e->data);
#endif
            break;

        default:
            break;
    }

    (void)handler_args;  // silence unused warnings
    (void)base;
}


/* ======================= Public API ======================= */
void ha_mqtt_init(const ha_mqtt_cfg_t *cfg) {
    if (cfg) s_cfg = *cfg; // shallow copy (const pointers expected to live)
    derive_ids_and_topics();
    s_boot_us = esp_timer_get_time();
    s_last_diag_us = 0;
}

void ha_mqtt_start(void) {
    if (s_client) return;

    esp_mqtt_client_config_t mc = {
        .broker.address.uri = s_cfg.broker_uri ? s_cfg.broker_uri : "mqtt://192.168.1.23",
        .credentials = {
            .username = s_cfg.username,
            .authentication.password = s_cfg.password
        },
        .session = {
            .keepalive = 30,   // <— add this
            .last_will = {
                .topic   = s_topic_status,
                .msg     = "offline",
                .qos     = 1,
                .retain  = 1
            }
        },
        .network.timeout_ms = 5000,
        // .network.reconnect_timeout_ms = 2000, // <— add if your IDF exposes it; otherwise omit
    };

    s_client = esp_mqtt_client_init(&mc);
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);
}


void ha_mqtt_stop(void) {
    if (!s_client) return;
    esp_mqtt_client_stop(s_client);
    esp_mqtt_client_destroy(s_client);
    s_client = NULL;
    s_connected = false;
}

void ha_mqtt_publish_presence(bool present, int distance_mm) {
    if (!s_connected) return;

    pub(s_topic_presence, present ? "ON" : "OFF", 0, 0);

    if (distance_mm >= 0) {
        char buf[16];
        float cm = distance_mm / 10.0f;
        snprintf(buf, sizeof(buf), "%.1f", cm);
        pub(s_topic_distance_cm, buf, 0, 0);
    }

    publish_periodic_diag_if_due();
}

void ha_mqtt_publish_rssi_now(void) {
    s_last_diag_us = 0; // force send on next call
    publish_periodic_diag_if_due();
}

void ha_mqtt_resend_discovery(void) {
    if (!s_connected) return;
    publish_discovery_all();
    publish_attrs_once();
    publish_periodic_diag_if_due();
}
