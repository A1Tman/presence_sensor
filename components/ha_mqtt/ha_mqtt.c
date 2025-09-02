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
#include "sensor_info.h"
#include <strings.h>

/* ======================= Build-time knobs ======================= */
#define DISTANCE_ENABLED_BY_DEFAULT         1   /* set to 1 once UART distance publishes */
#define MIN_DIAG_INTERVAL_SEC               30  /* uptime & RSSI cadence */

/* ======================= Module state ======================= */
static const char *TAG = "ha_mqtt";

static ha_mqtt_cfg_t s_cfg = {
    .broker_uri    = "mqtt://192.168.1.23",
    .username      = NULL,
    .password      = NULL,
    .friendly_name = "LD2420 Presence",
    .suggested_area= NULL,
    .app_version   = "0.2.0",
    .distance_supported = true,
    .legacy_node_ids = NULL,
    .legacy_node_ids_count = 0
};

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;
/* Failsafe: temporarily suppress HA discovery if it appears to cause disconnect loops */
static bool    s_discovery_suppressed = false;
static bool    s_did_discovery_this_session = false;
static int64_t s_last_connect_us = 0;
static int     s_disc_failures_recent = 0;
static int64_t s_disc_fail_window_start_us = 0;
/* Distance entity advertisement toggle */
static bool    s_distance_advertised = false;
/* Tunables for suppression */
#define DISC_FAIL_WINDOW_SEC 60
#define DISC_FAIL_MAX        3
#define DISC_QUICK_DC_SEC    5

/* Internal owned storage for dynamic strings */
static char s_broker_uri[128];
/* Owned storage for legacy node IDs (optional) */
#define MAX_LEGACY_IDS 8
static char s_legacy_id_buf[MAX_LEGACY_IDS][64];
static const char *s_legacy_id_ptrs[MAX_LEGACY_IDS];

/* Derived identifiers & topics */
static char s_mac_str[18];              // AA:BB:CC:DD:EE:FF
static char s_devid[32];                // presence-aabbcc
static char s_topic_base[64];           // presence/presence-aabbcc
static char s_topic_status[96];
static char s_topic_presence[96];
static char s_topic_distance_cm[96];
static char s_topic_attrs[96];
static char s_topic_rssi[96];
static char s_topic_uptime[96];
static char s_topic_dir_approach[96];
static char s_topic_dir_away[96];
static char s_topic_cmd_ld2420[96];
/* Cached state for reconnect */
static bool s_have_last = false;
static bool s_last_present = false;
static int  s_last_distance_mm = -1;
/* Config state + command topics */
static char s_topic_cfg_debounce_stat[96];
static char s_topic_cfg_holdon_stat[96];
static char s_topic_cfg_debounce_cmd[96];
static char s_topic_cfg_holdon_cmd[96];
static char s_topic_cfg_out_ah_stat[96];
static char s_topic_cfg_out_ah_cmd[96];
static char s_topic_cfg_out_pu_stat[96];
static char s_topic_cfg_out_pu_cmd[96];
static char s_topic_cfg_uartp_stat[96];
static char s_topic_cfg_uartp_cmd[96];
static char s_topic_cfg_psel_stat[96];
static char s_topic_cfg_psel_cmd[96];
static char s_topic_cfg_dth_stat[96];
static char s_topic_cfg_dth_cmd[96];
static char s_topic_cfg_distp_stat[96];
static char s_topic_cfg_distp_cmd[96];
/* LD2420 tuning (vendor params) */
static char s_topic_cfg_ld_min_stat[96];
static char s_topic_cfg_ld_min_cmd[96];
static char s_topic_cfg_ld_max_stat[96];
static char s_topic_cfg_ld_max_cmd[96];
static char s_topic_cfg_ld_delay_stat[96];
static char s_topic_cfg_ld_delay_cmd[96];
static char s_topic_cfg_ld_trig0_stat[96];
static char s_topic_cfg_ld_trig0_cmd[96];
static char s_topic_cfg_ld_hold0_stat[96];
static char s_topic_cfg_ld_hold0_cmd[96];
/* Attributes topics for number entities (for HA more-info tooltips) */
static char s_topic_cfg_ld_min_attr[96];
static char s_topic_cfg_ld_max_attr[96];
static char s_topic_cfg_ld_delay_attr[96];
static char s_topic_cfg_ld_trig0_attr[96];
static char s_topic_cfg_ld_hold0_attr[96];
/* Diagnostics topics */
static char s_topic_diag_out_raw[96];
static char s_topic_diag_out_active[96];
static char s_topic_diag_out_present[96];
static char s_topic_diag_uart_alive[96];
static char s_topic_diag_uart_baud[96];
/* Distance smoothing + zones */
static int  s_smooth_win = 5;          // samples (1..10)
static int  s_smooth_ring[16];         // mm values
static int  s_smooth_count = 0;
static int  s_smooth_head = 0;
static int  s_zone_min_cm[3] = {0, 51, 151};
static int  s_zone_max_cm[3] = {50, 150, 300};
static int  s_zone_last_on[3] = {0, 0, 0};
static char s_topic_zone_active[3][96];
static char s_topic_cfg_zone_min_stat[3][96];
static char s_topic_cfg_zone_min_cmd [3][96];
static char s_topic_cfg_zone_max_stat[3][96];
static char s_topic_cfg_zone_max_cmd [3][96];
static char s_topic_cfg_zone_min_attr[3][96];
static char s_topic_cfg_zone_max_attr[3][96];
static char s_topic_cfg_smooth_stat[96];
static char s_topic_cfg_smooth_cmd[96];
static char s_topic_cfg_smooth_attr[96];
/* Text sensors */
static char s_topic_sensor_ldfw[96];

/* HA Discovery topics */
static char s_disc_bs_presence[128];
static char s_disc_sensor_distance[128];
static char s_disc_sensor_rssi[128];
static char s_disc_sensor_uptime[128];
/* Discovery prefix (defaults to "homeassistant") */
static char s_disc_prefix[64] = "homeassistant";

/* Uptime ticker */
static int64_t s_boot_us = 0;
static int64_t s_last_diag_us = 0;  // publish diagnostics every ~60s

/* ======================= Utilities ======================= */
/* Forward declare to allow calls before definition */
static void publish_attrs_once(void);
static void mac_to_str(uint8_t mac[6], char out[18]) {
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void derive_ids_and_topics(void) {
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    mac_to_str(mac, s_mac_str);

    /* short hex id (last 3 bytes) */
    snprintf(s_devid, sizeof(s_devid), "presence-%02x%02x%02x", mac[3], mac[4], mac[5]);

    snprintf(s_topic_base, sizeof(s_topic_base), "presence/%s", s_devid);
    snprintf(s_topic_status, sizeof(s_topic_status),    "%s/status",      s_topic_base);
    snprintf(s_topic_presence, sizeof(s_topic_presence),"%s/presence",    s_topic_base);
    snprintf(s_topic_distance_cm, sizeof(s_topic_distance_cm), "%s/distance_cm", s_topic_base);
    snprintf(s_topic_attrs, sizeof(s_topic_attrs),      "%s/attributes",  s_topic_base);
    snprintf(s_topic_rssi, sizeof(s_topic_rssi),        "%s/rssi",        s_topic_base);
    snprintf(s_topic_uptime, sizeof(s_topic_uptime),    "%s/uptime_s",    s_topic_base);
    snprintf(s_topic_dir_approach, sizeof(s_topic_dir_approach), "%s/dir/approach", s_topic_base);
    snprintf(s_topic_dir_away,     sizeof(s_topic_dir_away),     "%s/dir/away",     s_topic_base);
    snprintf(s_topic_cmd_ld2420,   sizeof(s_topic_cmd_ld2420),   "%s/cmd/ld2420",   s_topic_base);
    snprintf(s_topic_cfg_debounce_stat, sizeof(s_topic_cfg_debounce_stat), "%s/cfg/debounce_ms", s_topic_base);
    snprintf(s_topic_cfg_holdon_stat,   sizeof(s_topic_cfg_holdon_stat),   "%s/cfg/hold_on_ms", s_topic_base);
    snprintf(s_topic_cfg_debounce_cmd,  sizeof(s_topic_cfg_debounce_cmd),  "%s/cmd/debounce_ms", s_topic_base);
    snprintf(s_topic_cfg_holdon_cmd,    sizeof(s_topic_cfg_holdon_cmd),    "%s/cmd/hold_on_ms",   s_topic_base);
    snprintf(s_topic_cfg_out_ah_stat,   sizeof(s_topic_cfg_out_ah_stat),   "%s/cfg/out_active_high", s_topic_base);
    snprintf(s_topic_cfg_out_ah_cmd,    sizeof(s_topic_cfg_out_ah_cmd),    "%s/cmd/out_active_high", s_topic_base);
    snprintf(s_topic_cfg_out_pu_stat,   sizeof(s_topic_cfg_out_pu_stat),   "%s/cfg/out_pullup", s_topic_base);
    snprintf(s_topic_cfg_out_pu_cmd,    sizeof(s_topic_cfg_out_pu_cmd),    "%s/cmd/out_pullup", s_topic_base);
    snprintf(s_topic_cfg_uartp_stat,    sizeof(s_topic_cfg_uartp_stat),    "%s/cfg/uart_presence", s_topic_base);
    snprintf(s_topic_cfg_uartp_cmd,     sizeof(s_topic_cfg_uartp_cmd),     "%s/cmd/uart_presence", s_topic_base);
    snprintf(s_topic_cfg_psel_stat,     sizeof(s_topic_cfg_psel_stat),     "%s/cfg/presence_source", s_topic_base);
    snprintf(s_topic_cfg_psel_cmd,      sizeof(s_topic_cfg_psel_cmd),      "%s/cmd/presence_source", s_topic_base);
    snprintf(s_topic_cfg_dth_stat,      sizeof(s_topic_cfg_dth_stat),      "%s/cfg/distance_thresh_cm", s_topic_base);
    snprintf(s_topic_cfg_dth_cmd,       sizeof(s_topic_cfg_dth_cmd),       "%s/cmd/distance_thresh_cm", s_topic_base);
    snprintf(s_topic_cfg_distp_stat,    sizeof(s_topic_cfg_distp_stat),    "%s/cfg/distance_presence", s_topic_base);
    snprintf(s_topic_cfg_distp_cmd,     sizeof(s_topic_cfg_distp_cmd),     "%s/cmd/distance_presence", s_topic_base);
    snprintf(s_topic_diag_out_raw,      sizeof(s_topic_diag_out_raw),      "%s/diag/out_raw", s_topic_base);
    snprintf(s_topic_diag_out_active,   sizeof(s_topic_diag_out_active),   "%s/diag/out_active", s_topic_base);
    snprintf(s_topic_diag_out_present,  sizeof(s_topic_diag_out_present),  "%s/diag/out_present", s_topic_base);
    snprintf(s_topic_diag_uart_alive,   sizeof(s_topic_diag_uart_alive),   "%s/diag/uart_alive", s_topic_base);
    snprintf(s_topic_diag_uart_baud,    sizeof(s_topic_diag_uart_baud),    "%s/diag/uart_baud", s_topic_base);
    /* Text sensor: LD2420 firmware */
    snprintf(s_topic_sensor_ldfw,       sizeof(s_topic_sensor_ldfw),       "%s/ld2420_fw", s_topic_base);

    /* LD2420 tuning (min/max gates, delay, thresholds index 0) */
    snprintf(s_topic_cfg_ld_min_stat,   sizeof(s_topic_cfg_ld_min_stat),   "%s/cfg/ld2420/min_gate", s_topic_base);
    snprintf(s_topic_cfg_ld_min_cmd,    sizeof(s_topic_cfg_ld_min_cmd),    "%s/cmd/ld2420/min_gate", s_topic_base);
    snprintf(s_topic_cfg_ld_max_stat,   sizeof(s_topic_cfg_ld_max_stat),   "%s/cfg/ld2420/max_gate", s_topic_base);
    snprintf(s_topic_cfg_ld_max_cmd,    sizeof(s_topic_cfg_ld_max_cmd),    "%s/cmd/ld2420/max_gate", s_topic_base);
    snprintf(s_topic_cfg_ld_delay_stat, sizeof(s_topic_cfg_ld_delay_stat), "%s/cfg/ld2420/delay_time", s_topic_base);
    snprintf(s_topic_cfg_ld_delay_cmd,  sizeof(s_topic_cfg_ld_delay_cmd),  "%s/cmd/ld2420/delay_time", s_topic_base);
    snprintf(s_topic_cfg_ld_trig0_stat, sizeof(s_topic_cfg_ld_trig0_stat), "%s/cfg/ld2420/trigger_00", s_topic_base);
    snprintf(s_topic_cfg_ld_trig0_cmd,  sizeof(s_topic_cfg_ld_trig0_cmd),  "%s/cmd/ld2420/trigger_00", s_topic_base);
    snprintf(s_topic_cfg_ld_hold0_stat, sizeof(s_topic_cfg_ld_hold0_stat), "%s/cfg/ld2420/maintain_00", s_topic_base);
    snprintf(s_topic_cfg_ld_hold0_cmd,  sizeof(s_topic_cfg_ld_hold0_cmd),  "%s/cmd/ld2420/maintain_00", s_topic_base);
    /* Attributes topics */
    snprintf(s_topic_cfg_ld_min_attr,   sizeof(s_topic_cfg_ld_min_attr),   "%s/attr/ld2420/min_gate", s_topic_base);
    snprintf(s_topic_cfg_ld_max_attr,   sizeof(s_topic_cfg_ld_max_attr),   "%s/attr/ld2420/max_gate", s_topic_base);
    snprintf(s_topic_cfg_ld_delay_attr, sizeof(s_topic_cfg_ld_delay_attr), "%s/attr/ld2420/delay_time", s_topic_base);
    snprintf(s_topic_cfg_ld_trig0_attr, sizeof(s_topic_cfg_ld_trig0_attr), "%s/attr/ld2420/trigger_00", s_topic_base);
    snprintf(s_topic_cfg_ld_hold0_attr, sizeof(s_topic_cfg_ld_hold0_attr), "%s/attr/ld2420/maintain_00", s_topic_base);

    /* Distance smoothing + zone topics */
    for (int i = 0; i < 3; ++i) {
        char id[8]; snprintf(id, sizeof(id), "%d", i+1);
        snprintf(s_topic_zone_active[i], sizeof(s_topic_zone_active[i]), "%s/zone/%s/active", s_topic_base, id);
        snprintf(s_topic_cfg_zone_min_stat[i], sizeof(s_topic_cfg_zone_min_stat[i]), "%s/cfg/zone/%s/min_cm", s_topic_base, id);
        snprintf(s_topic_cfg_zone_min_cmd[i],  sizeof(s_topic_cfg_zone_min_cmd[i]),  "%s/cmd/zone/%s/min_cm", s_topic_base, id);
        snprintf(s_topic_cfg_zone_max_stat[i], sizeof(s_topic_cfg_zone_max_stat[i]), "%s/cfg/zone/%s/max_cm", s_topic_base, id);
        snprintf(s_topic_cfg_zone_max_cmd[i],  sizeof(s_topic_cfg_zone_max_cmd[i]),  "%s/cmd/zone/%s/max_cm", s_topic_base, id);
        snprintf(s_topic_cfg_zone_min_attr[i], sizeof(s_topic_cfg_zone_min_attr[i]), "%s/attr/zone/%s/min_cm", s_topic_base, id);
        snprintf(s_topic_cfg_zone_max_attr[i], sizeof(s_topic_cfg_zone_max_attr[i]), "%s/attr/zone/%s/max_cm", s_topic_base, id);
    }
    snprintf(s_topic_cfg_smooth_stat, sizeof(s_topic_cfg_smooth_stat), "%s/cfg/distance_smoothing", s_topic_base);
    snprintf(s_topic_cfg_smooth_cmd,  sizeof(s_topic_cfg_smooth_cmd),  "%s/cmd/distance_smoothing", s_topic_base);
    snprintf(s_topic_cfg_smooth_attr, sizeof(s_topic_cfg_smooth_attr), "%s/attr/distance_smoothing", s_topic_base);

    /* Home Assistant discovery topics (retain) */
    snprintf(s_disc_bs_presence, sizeof(s_disc_bs_presence),
             "%s/binary_sensor/%s/presence/config", s_disc_prefix, s_devid);
    snprintf(s_disc_sensor_distance, sizeof(s_disc_sensor_distance),
             "%s/sensor/%s/distance/config", s_disc_prefix, s_devid);
    snprintf(s_disc_sensor_rssi, sizeof(s_disc_sensor_rssi),
             "%s/sensor/%s/rssi/config", s_disc_prefix, s_devid);
    snprintf(s_disc_sensor_uptime, sizeof(s_disc_sensor_uptime),
             "%s/sensor/%s/uptime/config", s_disc_prefix, s_devid);
}

static void pub(const char *topic, const char *payload, int qos, int retain) {
    if (!s_client || !s_connected) return;
    int mid = esp_mqtt_client_publish(s_client, topic, payload, 0, qos, retain);
    if (mid < 0) ESP_LOGW(TAG, "Publish failed to %s", topic);
}

/* ======================= Discovery payloads ======================= */
/* Publish empty retained configs for legacy node_ids so HA removes them */
static void cleanup_legacy_discovery(void) {
    if (!s_client || !s_connected) return;
    if (!s_cfg.legacy_node_ids || s_cfg.legacy_node_ids_count <= 0) return;

    for (int i = 0; i < s_cfg.legacy_node_ids_count; i++) {
        const char *lid = s_cfg.legacy_node_ids[i];
        if (!lid || !lid[0]) continue;
        if (strcmp(lid, s_devid) == 0) continue; // don't clear current node id

        char topic[192];
        // Binary sensors
        snprintf(topic, sizeof(topic), "%s/binary_sensor/%s/presence/config",       s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/binary_sensor/%s/dir_approach/config",  s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/binary_sensor/%s/dir_away/config",      s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/binary_sensor/%s/out_raw/config",       s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/binary_sensor/%s/out_active/config",    s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/binary_sensor/%s/uart_alive/config",    s_disc_prefix, lid); pub(topic, "", 1, 1);

        // Sensors
        snprintf(topic, sizeof(topic), "%s/sensor/%s/distance/config",             s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/sensor/%s/rssi/config",                 s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/sensor/%s/uptime/config",               s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/sensor/%s/uart_baud/config",            s_disc_prefix, lid); pub(topic, "", 1, 1);

        // Numbers
        snprintf(topic, sizeof(topic), "%s/number/%s/debounce/config",             s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/number/%s/hold_on/config",              s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/number/%s/distance_thresh/config",      s_disc_prefix, lid); pub(topic, "", 1, 1);

        // Select
        snprintf(topic, sizeof(topic), "%s/select/%s/presence_source/config",      s_disc_prefix, lid); pub(topic, "", 1, 1);

        // Switches
        snprintf(topic, sizeof(topic), "%s/switch/%s/distance_presence/config",    s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/switch/%s/out_active_high/config",      s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/switch/%s/out_pullup/config",           s_disc_prefix, lid); pub(topic, "", 1, 1);
        snprintf(topic, sizeof(topic), "%s/switch/%s/uart_presence/config",        s_disc_prefix, lid); pub(topic, "", 1, 1);
    }
}

static void publish_discovery_all(void) {
    char dev_block[512];
    char area[96] = {0};
    const char *dev_name = s_cfg.friendly_name ? s_cfg.friendly_name : "LD2420 Presence";

    if (s_cfg.suggested_area && s_cfg.suggested_area[0]) {
        snprintf(area, sizeof(area), ",\"suggested_area\":\"%s\"", s_cfg.suggested_area);
    }

    /* Device object (only here; entities keep generic names) */
    const char *model = s_cfg.device_model ? s_cfg.device_model : "ESP32 Presence Sensor";
    snprintf(dev_block, sizeof(dev_block),
        "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"Hi-Link + DIY\","
        "\"mdl\":\"%s\",\"sw\":\"%s\","
        "\"connections\":[[\"mac\",\"%s\"]]}%s",
        s_devid, dev_name,
        model,
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

    /* Direction: Approach (binary_sensor) */
    {
        char payload[1024]; int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Approach\","
            "\"uniq_id\":\"%s_dir_approach\","
            "\"stat_t\":\"%s\","
            "\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"obj_id\":\"dir_approach\",",
            s_devid, s_topic_dir_approach, s_topic_status
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192];
        snprintf(disc, sizeof(disc), "%s/binary_sensor/%s/dir_approach/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }
    /* Direction: Away (binary_sensor) */
    {
        char payload[1024]; int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Away\","
            "\"uniq_id\":\"%s_dir_away\","
            "\"stat_t\":\"%s\","
            "\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"obj_id\":\"dir_away\",",
            s_devid, s_topic_dir_away, s_topic_status
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192];
        snprintf(disc, sizeof(disc), "%s/binary_sensor/%s/dir_away/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* Distance (sensor) — cm; advertise only if supported */
    if (s_cfg.distance_supported) {
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
    } else {
        /* Clear any previously retained distance config so HA removes the entity */
        pub(s_disc_sensor_distance, "", 1, 1);
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

    /* LD2420 Firmware (text sensor) */
    {
        char payload[1024]; int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"LD2420 Firmware\","
            "\"uniq_id\":\"%s_ld2420_fw\","
            "\"stat_t\":\"%s\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"ent_cat\":\"diagnostic\",\"obj_id\":\"ld2420_fw\",",
            s_devid, s_topic_sensor_ldfw, s_topic_status);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/sensor/%s/ld2420_fw/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
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

    /* OUT diagnostics omitted for LD2420; keep only UART diagnostics */
    /* UART Alive (binary_sensor) */
    {
        char payload[1024]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"UART Alive\",\"uniq_id\":\"%s_uart_alive\",\"stat_t\":\"%s\",\"obj_id\":\"uart_alive\",",
            s_devid, s_topic_diag_uart_alive);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[128]; snprintf(disc, sizeof(disc), "%s/binary_sensor/%s/uart_alive/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }
    /* UART Baud (sensor) */
    {
        char payload[1024]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"UART Baud\",\"uniq_id\":\"%s_uart_baud\",\"stat_t\":\"%s\",\"unit_of_meas\":\"baud\",\"obj_id\":\"uart_baud\",",
            s_devid, s_topic_diag_uart_baud);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[128]; snprintf(disc, sizeof(disc), "%s/sensor/%s/uart_baud/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* LD2420 numbers: min gate (0..15) */
    {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Min Gate\",\"uniq_id\":\"%s_ld_min_gate\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"json_attr_t\":\"%s\",\"min\":0,\"max\":15,\"step\":1,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_ld_min_cmd, s_topic_cfg_ld_min_stat, s_topic_cfg_ld_min_attr);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/ld_min_gate/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }
    /* LD2420 numbers: max gate (0..15) */
    {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Max Gate\",\"uniq_id\":\"%s_ld_max_gate\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"json_attr_t\":\"%s\",\"min\":0,\"max\":15,\"step\":1,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_ld_max_cmd, s_topic_cfg_ld_max_stat, s_topic_cfg_ld_max_attr);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/ld_max_gate/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }
    /* LD2420 numbers: target delay (0..65535) */
    {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Target Delay\",\"uniq_id\":\"%s_ld_delay\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"json_attr_t\":\"%s\",\"min\":0,\"max\":65535,\"step\":1,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_ld_delay_cmd, s_topic_cfg_ld_delay_stat, s_topic_cfg_ld_delay_attr);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/ld_delay/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }
    /* LD2420 numbers: trigger threshold 00 (0..65535) */
    {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Trigger Th 00\",\"uniq_id\":\"%s_ld_trig00\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"json_attr_t\":\"%s\",\"min\":0,\"max\":65535,\"step\":1,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_ld_trig0_cmd, s_topic_cfg_ld_trig0_stat, s_topic_cfg_ld_trig0_attr);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/ld_trig00/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }
    /* LD2420 numbers: maintain threshold 00 (0..65535) */
    {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Maintain Th 00\",\"uniq_id\":\"%s_ld_hold00\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"json_attr_t\":\"%s\",\"min\":0,\"max\":65535,\"step\":1,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_ld_hold0_cmd, s_topic_cfg_ld_hold0_stat, s_topic_cfg_ld_hold0_attr);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/ld_hold00/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* Zone binary_sensors: Zone 1..3 */
    for (int i = 0; i < 3; ++i) {
        char payload[1024]; int len=0; char id[8]; snprintf(id, sizeof(id), "%d", i+1);
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Zone %s\",\"uniq_id\":\"%s_zone_%s\",\"stat_t\":\"%s\",\"avty_t\":\"%s\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"obj_id\":\"zone_%s\",",
            id, s_devid, id, s_topic_zone_active[i], s_topic_status, id);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/binary_sensor/%s/zone_%s/config", s_disc_prefix, s_devid, id);
        pub(disc, payload, 1, 1);
    }

    /* Zone number sliders and Distance Smoothing */
    for (int i = 0; i < 3; ++i) {
        char id[8]; snprintf(id, sizeof(id), "%d", i+1);
        // Min
        {
            char payload[512]; int len=0;
            len += snprintf(payload+len, sizeof(payload)-len,
                "{\"name\":\"Zone %s Min (cm)\",\"uniq_id\":\"%s_zone%s_min\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"json_attr_t\":\"%s\",\"min\":0,\"max\":600,\"step\":1,\"mode\":\"slider\",\"ent_cat\":\"config\",",
                id, s_devid, id, s_topic_cfg_zone_min_cmd[i], s_topic_cfg_zone_min_stat[i], s_topic_cfg_zone_min_attr[i]);
            len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
            len += snprintf(payload+len, sizeof(payload)-len, "}");
            char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/zone%s_min/config", s_disc_prefix, s_devid, id);
            pub(disc, payload, 1, 1);
        }
        // Max
        {
            char payload[512]; int len=0;
            len += snprintf(payload+len, sizeof(payload)-len,
                "{\"name\":\"Zone %s Max (cm)\",\"uniq_id\":\"%s_zone%s_max\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"json_attr_t\":\"%s\",\"min\":0,\"max\":600,\"step\":1,\"mode\":\"slider\",\"ent_cat\":\"config\",",
                id, s_devid, id, s_topic_cfg_zone_max_cmd[i], s_topic_cfg_zone_max_stat[i], s_topic_cfg_zone_max_attr[i]);
            len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
            len += snprintf(payload+len, sizeof(payload)-len, "}");
            char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/zone%s_max/config", s_disc_prefix, s_devid, id);
            pub(disc, payload, 1, 1);
        }
    }
    // Smoothing
    {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Distance Smoothing\",\"uniq_id\":\"%s_dist_smooth\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"json_attr_t\":\"%s\",\"min\":1,\"max\":10,\"step\":1,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_smooth_cmd, s_topic_cfg_smooth_stat, s_topic_cfg_smooth_attr);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/distance_smoothing/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* Debounce (number) — configurable */
    if (s_cfg.set_debounce_ms && s_cfg.get_debounce_ms) {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Debounce\","
            "\"uniq_id\":\"%s_debounce\","
            "\"stat_t\":\"%s\",\"cmd_t\":\"%s\","
            "\"unit_of_meas\":\"ms\",\"mode\":\"slider\",\"ent_cat\":\"config\","
            "\"min\":50,\"max\":3000,\"step\":10,"
            "\"obj_id\":\"debounce_ms\",",
            s_devid, s_topic_cfg_debounce_stat, s_topic_cfg_debounce_cmd
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[128];
        snprintf(disc, sizeof(disc), "%s/number/%s/debounce/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* Hold On (number) — configurable */
    if (s_cfg.set_hold_on_ms && s_cfg.get_hold_on_ms) {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Hold On\","
            "\"uniq_id\":\"%s_hold_on\","
            "\"stat_t\":\"%s\",\"cmd_t\":\"%s\","
            "\"unit_of_meas\":\"ms\",\"mode\":\"slider\",\"ent_cat\":\"config\","
            "\"min\":500,\"max\":15000,\"step\":50,"
            "\"obj_id\":\"hold_on_ms\",",
            s_devid, s_topic_cfg_holdon_stat, s_topic_cfg_holdon_cmd
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[128];
        snprintf(disc, sizeof(disc), "%s/number/%s/hold_on/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* Presence Source (select) */
    if (s_cfg.set_presence_source && s_cfg.get_presence_source) {
        char payload[1024]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Presence Source\","
            "\"uniq_id\":\"%s_presence_src\","
            "\"stat_t\":\"%s\",\"cmd_t\":\"%s\","
            "\"options\":[\"out\",\"uart\",\"combined\",\"distance\"],"
            "\"obj_id\":\"presence_source\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_psel_stat, s_topic_cfg_psel_cmd);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[128]; snprintf(disc, sizeof(disc), "%s/select/%s/presence_source/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* Distance threshold (number cm) */
    if (s_cfg.set_distance_thresh_cm && s_cfg.get_distance_thresh_cm) {
        char payload[1024]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Distance Threshold\","
            "\"uniq_id\":\"%s_dist_thresh\","
            "\"stat_t\":\"%s\",\"cmd_t\":\"%s\","
            "\"unit_of_meas\":\"cm\",\"mode\":\"slider\",\"ent_cat\":\"config\","
            "\"min\":10,\"max\":600,\"step\":10,"
            "\"obj_id\":\"distance_thresh_cm\",",
            s_devid, s_topic_cfg_dth_stat, s_topic_cfg_dth_cmd);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[128]; snprintf(disc, sizeof(disc), "%s/number/%s/distance_thresh/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* Distance presence enable (switch) */
    if (s_cfg.set_distance_presence_enable && s_cfg.get_distance_presence_enable) {
        char payload[1024]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Distance Presence\","
            "\"uniq_id\":\"%s_dist_presence\","
            "\"stat_t\":\"%s\",\"cmd_t\":\"%s\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
            "\"obj_id\":\"distance_presence\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_distp_stat, s_topic_cfg_distp_cmd);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[128]; snprintf(disc, sizeof(disc), "%s/switch/%s/distance_presence/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* OUT Active High (switch) */
    if (s_cfg.set_out_active_high && s_cfg.get_out_active_high) {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"OUT Active High\","
            "\"uniq_id\":\"%s_out_ah\","
            "\"stat_t\":\"%s\",\"cmd_t\":\"%s\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
            "\"obj_id\":\"out_active_high\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_out_ah_stat, s_topic_cfg_out_ah_cmd);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[128]; snprintf(disc, sizeof(disc), "%s/switch/%s/out_active_high/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* OUT Pull-up (switch) */
    if (s_cfg.set_out_pullup_enabled && s_cfg.get_out_pullup_enabled) {
        char payload[1024]; int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"OUT Pull-up\","
            "\"uniq_id\":\"%s_out_pu\","
            "\"stat_t\":\"%s\",\"cmd_t\":\"%s\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
            "\"obj_id\":\"out_pullup\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_out_pu_stat, s_topic_cfg_out_pu_cmd);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[128]; snprintf(disc, sizeof(disc), "%s/switch/%s/out_pullup/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* UART Presence (switch) */
    if (s_cfg.set_uart_presence_enabled && s_cfg.get_uart_presence_enabled) {
        char payload[1024]; int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"UART Presence\","
            "\"uniq_id\":\"%s_uart_presence\","
            "\"stat_t\":\"%s\",\"cmd_t\":\"%s\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
            "\"obj_id\":\"uart_presence\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_uartp_stat, s_topic_cfg_uartp_cmd);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[128]; snprintf(disc, sizeof(disc), "%s/switch/%s/uart_presence/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
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

    /* If sensor FW just became available, refresh attributes */
    static char s_last_fw_published[64];
    const char *fw = sensor_info_get_fw();
    if (fw && fw[0] && strcmp(fw, s_last_fw_published) != 0) {
        publish_attrs_once();
        size_t n = strlen(fw);
        if (n >= sizeof(s_last_fw_published)) {
            n = sizeof(s_last_fw_published)-1;
        }
        memcpy(s_last_fw_published, fw, n);
        s_last_fw_published[n] = '\0';
        /* Publish to text sensor as well */
        pub(s_topic_sensor_ldfw, fw, 0, 1);
    }
}

/* Attributes blob (mac, fw, ip, etc.) */
static void publish_attrs_once(void) {
    esp_netif_ip_info_t ip;
    char json[512];
    char ip_str[32] = "0.0.0.0";
    const char *sensor_fw = sensor_info_get_fw();

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip.ip));
    }
    const char *model = s_cfg.device_model ? s_cfg.device_model : "ESP32 Presence Sensor";
    int len = 0;
    len += snprintf(json+len, sizeof(json)-len, "{");
    len += snprintf(json+len, sizeof(json)-len, "\"mac\":\"%s\",", s_mac_str);
    len += snprintf(json+len, sizeof(json)-len, "\"device_id\":\"%s\",", s_devid);
    len += snprintf(json+len, sizeof(json)-len, "\"model\":\"%s\",", model);
    len += snprintf(json+len, sizeof(json)-len, "\"sw_version\":\"%s\",",
                    s_cfg.app_version ? s_cfg.app_version : "0.0.0");
    len += snprintf(json+len, sizeof(json)-len, "\"ip\":\"%s\"", ip_str);
    if (sensor_fw && sensor_fw[0]) {
        len += snprintf(json+len, sizeof(json)-len, ",\"sensor_fw\":\"%s\"", sensor_fw);
    }
    len += snprintf(json+len, sizeof(json)-len, "}");
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
            s_last_connect_us = esp_timer_get_time();
            s_did_discovery_this_session = false;
            // Proactively clean legacy retained discovery topics (if any) and publish discovery
            if (!s_discovery_suppressed) {
                cleanup_legacy_discovery();
                publish_discovery_all();
                s_did_discovery_this_session = true;
            } else {
                ESP_LOGW(TAG, "HA discovery suppressed due to prior quick disconnects; check broker ACLs");
            }
            publish_birth_online();
        publish_attrs_once();
        publish_periodic_diag_if_due();
            /* Publish initial switch states and subscribe */
            if (s_cfg.set_out_active_high && s_cfg.get_out_active_high) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_out_ah_cmd, 1);
                pub(s_topic_cfg_out_ah_stat, s_cfg.get_out_active_high() ? "ON" : "OFF", 1, 1);
            }
            if (s_cfg.set_out_pullup_enabled && s_cfg.get_out_pullup_enabled) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_out_pu_cmd, 1);
                pub(s_topic_cfg_out_pu_stat, s_cfg.get_out_pullup_enabled() ? "ON" : "OFF", 1, 1);
            }
            if (s_cfg.set_uart_presence_enabled && s_cfg.get_uart_presence_enabled) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_uartp_cmd, 1);
                pub(s_topic_cfg_uartp_stat, s_cfg.get_uart_presence_enabled() ? "ON" : "OFF", 1, 1);
            }
            if (s_cfg.ld2420_handle_cmd) {
                esp_mqtt_client_subscribe(s_client, s_topic_cmd_ld2420, 1);
                ESP_LOGI(TAG, "Subscribed LD2420 cmd: %s", s_topic_cmd_ld2420);
                /* Proactively query firmware version once per connection */
                const char *ver = "version";
                s_cfg.ld2420_handle_cmd(ver, (int)strlen(ver));
            }
            /* Re-send last presence so HA reflects current state after reconnect */
            if (s_have_last) {
                ha_mqtt_publish_presence(s_last_present, s_last_distance_mm);
            }
            /* Subscribe to config command topics and publish current states */
            if (s_cfg.set_debounce_ms && s_cfg.get_debounce_ms) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_debounce_cmd, 1);
                char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_debounce_ms());
                pub(s_topic_cfg_debounce_stat, buf, 1, 1);
            }
            if (s_cfg.set_hold_on_ms && s_cfg.get_hold_on_ms) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_holdon_cmd, 1);
                char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_hold_on_ms());
                pub(s_topic_cfg_holdon_stat, buf, 1, 1);
            }
            if (s_cfg.set_presence_source && s_cfg.get_presence_source) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_psel_cmd, 1);
                const char *opt = "out"; int ps = s_cfg.get_presence_source();
                if (ps==1) opt="uart"; else if (ps==2) opt="combined"; else if (ps==3) opt="distance";
                pub(s_topic_cfg_psel_stat, opt, 1, 1);
            }
            if (s_cfg.set_distance_thresh_cm && s_cfg.get_distance_thresh_cm) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_dth_cmd, 1);
                char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_distance_thresh_cm());
                pub(s_topic_cfg_dth_stat, buf, 1, 1);
            }
            if (s_cfg.set_distance_presence_enable && s_cfg.get_distance_presence_enable) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_distp_cmd, 1);
                pub(s_topic_cfg_distp_stat, s_cfg.get_distance_presence_enable()?"ON":"OFF", 1, 1);
            }
            /* Subscribe LD2420 tuning number commands and publish initial state (optimistic) */
            esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_min_cmd, 1);
            pub(s_topic_cfg_ld_min_stat, "0", 1, 1);
            pub(s_topic_cfg_ld_min_attr,  "{\"help\":\"Minimum distance gate (0–15). One gate ≈ 0.7 m.\"}", 1, 1);
            esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_max_cmd, 1);
            pub(s_topic_cfg_ld_max_stat, "0", 1, 1);
            pub(s_topic_cfg_ld_max_attr,  "{\"help\":\"Maximum distance gate (0–15). Must be ≥ Min Gate.\"}", 1, 1);
            esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_delay_cmd, 1);
            pub(s_topic_cfg_ld_delay_stat, "0", 1, 1);
            pub(s_topic_cfg_ld_delay_attr,"{\"help\":\"Target disappearance delay T (firmware units). Larger = longer time to clear.\"}", 1, 1);
            esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_trig0_cmd, 1);
            pub(s_topic_cfg_ld_trig0_stat, "0", 1, 1);
            pub(s_topic_cfg_ld_trig0_attr,"{\"help\":\"Trigger sensitivity (index 00). Recommend >5× noise.\"}", 1, 1);
            esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_hold0_cmd, 1);
            pub(s_topic_cfg_ld_hold0_stat, "0", 1, 1);
            pub(s_topic_cfg_ld_hold0_attr,"{\"help\":\"Maintain (micro‑motion) sensitivity (index 00). Recommend 2–5× noise.\"}", 1, 1);
            /* Zones + smoothing defaults */
            for (int i = 0; i < 3; ++i) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_zone_min_cmd[i], 1);
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_zone_max_cmd[i], 1);
                char v[8]; snprintf(v, sizeof(v), "%d", s_zone_min_cm[i]); pub(s_topic_cfg_zone_min_stat[i], v, 1, 1);
                snprintf(v, sizeof(v), "%d", s_zone_max_cm[i]); pub(s_topic_cfg_zone_max_stat[i], v, 1, 1);
                pub(s_topic_cfg_zone_min_attr[i],  "{\"help\":\"Zone minimum distance (cm).\"}", 1, 1);
                pub(s_topic_cfg_zone_max_attr[i],  "{\"help\":\"Zone maximum distance (cm).\"}", 1, 1);
            }
            esp_mqtt_client_subscribe(s_client, s_topic_cfg_smooth_cmd, 1);
            { char v[8]; snprintf(v, sizeof(v), "%d", s_smooth_win); pub(s_topic_cfg_smooth_stat, v, 1, 1); }
            pub(s_topic_cfg_smooth_attr, "{\"help\":\"Moving average window (samples).\"}", 1, 1);
            break;

        case MQTT_EVENT_DISCONNECTED: {
            s_connected = false;
            ESP_LOGW(TAG, "MQTT disconnected");
            /* If we disconnected shortly after a connection where we attempted discovery,
               count it toward suppression to avoid loops on brokers that forbid HA topics. */
            int64_t now = esp_timer_get_time();
            if (s_did_discovery_this_session && (now - s_last_connect_us) < (int64_t)DISC_QUICK_DC_SEC * 1000000LL) {
                if (s_disc_fail_window_start_us == 0 ||
                    (now - s_disc_fail_window_start_us) > (int64_t)DISC_FAIL_WINDOW_SEC * 1000000LL) {
                    s_disc_fail_window_start_us = now;
                    s_disc_failures_recent = 0;
                }
                s_disc_failures_recent++;
                if (s_disc_failures_recent >= DISC_FAIL_MAX) {
                    s_discovery_suppressed = true;
                    ESP_LOGW(TAG, "Suppressing HA discovery after %d quick disconnects in %ds window", s_disc_failures_recent, DISC_FAIL_WINDOW_SEC);
                }
            }
            break;
        }

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
            if (e && e->topic && e->data) {
                /* Handle config commands */
                const char *t = e->topic; int tlen = e->topic_len;
                if (s_cfg.set_debounce_ms && tlen == (int)strlen(s_topic_cfg_debounce_cmd) && strncmp(t, s_topic_cfg_debounce_cmd, tlen) == 0) {
                    char tmp[16]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n] = '\0';
                    int v = atoi(tmp);
                    if (v < 0) v = 0;
                    if (v > 3000) v = 3000;
                    s_cfg.set_debounce_ms(v);
                    char buf[16]; snprintf(buf, sizeof(buf), "%d", v); pub(s_topic_cfg_debounce_stat, buf, 1, 1);
                } else if (s_cfg.set_hold_on_ms && tlen == (int)strlen(s_topic_cfg_holdon_cmd) && strncmp(t, s_topic_cfg_holdon_cmd, tlen) == 0) {
                    char tmp[16]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n] = '\0';
                    int v = atoi(tmp);
                    if (v < 0) v = 0;
                    if (v > 15000) v = 15000;
                    s_cfg.set_hold_on_ms(v);
                    char buf[16]; snprintf(buf, sizeof(buf), "%d", v); pub(s_topic_cfg_holdon_stat, buf, 1, 1);
                } else if (s_cfg.set_out_active_high && tlen == (int)strlen(s_topic_cfg_out_ah_cmd) && strncmp(t, s_topic_cfg_out_ah_cmd, tlen) == 0) {
                    bool on = (e->data_len >= 2 && strncasecmp(e->data, "ON", 2) == 0);
                    s_cfg.set_out_active_high(on);
                    pub(s_topic_cfg_out_ah_stat, on ? "ON" : "OFF", 1, 1);
                } else if (s_cfg.set_out_pullup_enabled && tlen == (int)strlen(s_topic_cfg_out_pu_cmd) && strncmp(t, s_topic_cfg_out_pu_cmd, tlen) == 0) {
                    bool on = (e->data_len >= 2 && strncasecmp(e->data, "ON", 2) == 0);
                    s_cfg.set_out_pullup_enabled(on);
                    pub(s_topic_cfg_out_pu_stat, on ? "ON" : "OFF", 1, 1);
                } else if (s_cfg.set_uart_presence_enabled && tlen == (int)strlen(s_topic_cfg_uartp_cmd) && strncmp(t, s_topic_cfg_uartp_cmd, tlen) == 0) {
                    bool on = (e->data_len >= 2 && strncasecmp(e->data, "ON", 2) == 0);
                    s_cfg.set_uart_presence_enabled(on);
                    pub(s_topic_cfg_uartp_stat, on ? "ON" : "OFF", 1, 1);
                } else if (s_cfg.set_presence_source && tlen == (int)strlen(s_topic_cfg_psel_cmd) && strncmp(t, s_topic_cfg_psel_cmd, tlen) == 0) {
                    char tmp[32]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n] = '\0';
                    int ps = 0;
                    if (!strncasecmp(tmp,"uart",4)) ps=1;
                    else if (!strncasecmp(tmp,"combined",8)) ps=2;
                    else if (!strncasecmp(tmp,"distance",8)) ps=3;
                    else ps=0;
                    s_cfg.set_presence_source(ps);
                    const char *opt = ps==1?"uart": ps==2?"combined": ps==3?"distance":"out";
                    pub(s_topic_cfg_psel_stat, opt, 1, 1);
                } else if (s_cfg.set_distance_thresh_cm && tlen == (int)strlen(s_topic_cfg_dth_cmd) && strncmp(t, s_topic_cfg_dth_cmd, tlen) == 0) {
                    char tmp[16]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n] = '\0';
                    int v = atoi(tmp);
                    if (v < 10) v = 10;
                    if (v > 600) v = 600;
                    s_cfg.set_distance_thresh_cm(v);
                    char buf[16]; snprintf(buf, sizeof(buf), "%d", v); pub(s_topic_cfg_dth_stat, buf, 1, 1);
                } else if (s_cfg.set_distance_presence_enable && tlen == (int)strlen(s_topic_cfg_distp_cmd) && strncmp(t, s_topic_cfg_distp_cmd, tlen) == 0) {
                    bool on = (e->data_len >= 2 && strncasecmp(e->data, "ON", 2) == 0);
                    s_cfg.set_distance_presence_enable(on);
                    pub(s_topic_cfg_distp_stat, on ? "ON" : "OFF", 1, 1);
                } else if (s_cfg.ld2420_handle_cmd && tlen == (int)strlen(s_topic_cmd_ld2420) && strncmp(t, s_topic_cmd_ld2420, tlen) == 0) {
                    s_cfg.ld2420_handle_cmd(e->data, e->data_len);
                } else if (s_cfg.ld2420_handle_cmd && tlen == (int)strlen(s_topic_cfg_ld_min_cmd) && strncmp(t, s_topic_cfg_ld_min_cmd, tlen) == 0) {
                    char tmp[8]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n]='\0';
                    int v = atoi(tmp);
                    if (v<0) v=0;
                    if (v>15) v=15;
                    char cmd[32]; snprintf(cmd, sizeof(cmd), "set min %d", v);
                    s_cfg.ld2420_handle_cmd(cmd, (int)strlen(cmd));
                    char val[8]; snprintf(val, sizeof(val), "%d", v); pub(s_topic_cfg_ld_min_stat, val, 1, 1);
                } else if (s_cfg.ld2420_handle_cmd && tlen == (int)strlen(s_topic_cfg_ld_max_cmd) && strncmp(t, s_topic_cfg_ld_max_cmd, tlen) == 0) {
                    char tmp[8]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n]='\0';
                    int v = atoi(tmp);
                    if (v<0) v=0;
                    if (v>15) v=15;
                    char cmd[32]; snprintf(cmd, sizeof(cmd), "set max %d", v);
                    s_cfg.ld2420_handle_cmd(cmd, (int)strlen(cmd));
                    char val[8]; snprintf(val, sizeof(val), "%d", v); pub(s_topic_cfg_ld_max_stat, val, 1, 1);
                } else if (s_cfg.ld2420_handle_cmd && tlen == (int)strlen(s_topic_cfg_ld_delay_cmd) && strncmp(t, s_topic_cfg_ld_delay_cmd, tlen) == 0) {
                    char tmp[16]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n]='\0';
                    unsigned v = (unsigned)strtoul(tmp, NULL, 10);
                    if (v>65535) v=65535;
                    char cmd[48]; snprintf(cmd, sizeof(cmd), "set delay %u", v);
                    s_cfg.ld2420_handle_cmd(cmd, (int)strlen(cmd));
                    char val[16]; snprintf(val, sizeof(val), "%u", v); pub(s_topic_cfg_ld_delay_stat, val, 1, 1);
                } else if (s_cfg.ld2420_handle_cmd && tlen == (int)strlen(s_topic_cfg_ld_trig0_cmd) && strncmp(t, s_topic_cfg_ld_trig0_cmd, tlen) == 0) {
                    char tmp[16]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n]='\0';
                    unsigned v = (unsigned)strtoul(tmp, NULL, 10);
                    if (v>65535) v=65535;
                    char cmd[64]; snprintf(cmd, sizeof(cmd), "set trig 0 %u", v);
                    s_cfg.ld2420_handle_cmd(cmd, (int)strlen(cmd));
                    char val[16]; snprintf(val, sizeof(val), "%u", v); pub(s_topic_cfg_ld_trig0_stat, val, 1, 1);
                } else if (s_cfg.ld2420_handle_cmd && tlen == (int)strlen(s_topic_cfg_ld_hold0_cmd) && strncmp(t, s_topic_cfg_ld_hold0_cmd, tlen) == 0) {
                    char tmp[16]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n]='\0';
                    unsigned v = (unsigned)strtoul(tmp, NULL, 10);
                    if (v>65535) v=65535;
                    char cmd[64]; snprintf(cmd, sizeof(cmd), "set hold 0 %u", v);
                    s_cfg.ld2420_handle_cmd(cmd, (int)strlen(cmd));
                    char val[16]; snprintf(val, sizeof(val), "%u", v); pub(s_topic_cfg_ld_hold0_stat, val, 1, 1);
                } else {
                    /* Zones and smoothing */
                    for (int i = 0; i < 3; ++i) {
                        if (tlen == (int)strlen(s_topic_cfg_zone_min_cmd[i]) && strncmp(t, s_topic_cfg_zone_min_cmd[i], tlen) == 0) {
                            char tmp[16]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n]='\0';
                            int v = atoi(tmp);
                            if (v < 0) v = 0;
                            if (v > 600) v = 600;
                            s_zone_min_cm[i] = v;
                            char val[8]; snprintf(val, sizeof(val), "%d", v); pub(s_topic_cfg_zone_min_stat[i], val, 1, 1);
                            break;
                        }
                        if (tlen == (int)strlen(s_topic_cfg_zone_max_cmd[i]) && strncmp(t, s_topic_cfg_zone_max_cmd[i], tlen) == 0) {
                            char tmp[16]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n]='\0';
                            int v = atoi(tmp);
                            if (v < 0) v = 0;
                            if (v > 600) v = 600;
                            s_zone_max_cm[i] = v;
                            char val[8]; snprintf(val, sizeof(val), "%d", v); pub(s_topic_cfg_zone_max_stat[i], val, 1, 1);
                            break;
                        }
                    }
                    if (tlen == (int)strlen(s_topic_cfg_smooth_cmd) && strncmp(t, s_topic_cfg_smooth_cmd, tlen) == 0) {
                        char tmp[8]; int n = e->data_len < (int)sizeof(tmp)-1 ? e->data_len : (int)sizeof(tmp)-1; memcpy(tmp, e->data, n); tmp[n]='\0';
                        int v = atoi(tmp);
                        if (v < 1) v = 1;
                        if (v > 10) v = 10;
                        s_smooth_win = v;
                        s_smooth_count = 0; s_smooth_head = 0; // reset buffer
                        char val[8]; snprintf(val, sizeof(val), "%d", v); pub(s_topic_cfg_smooth_stat, val, 1, 1);
                    }
                }
            }
            break;

        default:
            break;
    }

    (void)handler_args;  // silence unused warnings
    (void)base;
}


/* ======================= Public API ======================= */
void ha_mqtt_init(const ha_mqtt_cfg_t *cfg) {
    if (cfg) s_cfg = *cfg; // shallow copy (we may override broker_uri to internal buffer)

    /* Copy broker URI into owned storage if provided */
    if (cfg && cfg->broker_uri && cfg->broker_uri[0]) {
        snprintf(s_broker_uri, sizeof(s_broker_uri), "%s", cfg->broker_uri);
        s_cfg.broker_uri = s_broker_uri;
    }
    /* Discovery prefix override */
    if (cfg && cfg->discovery_prefix && cfg->discovery_prefix[0]) {
        snprintf(s_disc_prefix, sizeof(s_disc_prefix), "%s", cfg->discovery_prefix);
    } else {
        snprintf(s_disc_prefix, sizeof(s_disc_prefix), "%s", "homeassistant");
    }
    /* Copy legacy IDs into owned storage to avoid dangling pointers */
    int n_legacy = 0;
    if (cfg && cfg->legacy_node_ids && cfg->legacy_node_ids_count > 0) {
        int lim = cfg->legacy_node_ids_count;
        if (lim > MAX_LEGACY_IDS) lim = MAX_LEGACY_IDS;
        for (int i = 0; i < lim; ++i) {
            const char *src = cfg->legacy_node_ids[i];
            if (!src || !src[0]) continue;
            size_t L = strlen(src);
            if (L >= sizeof(s_legacy_id_buf[0])) L = sizeof(s_legacy_id_buf[0]) - 1;
            memcpy(s_legacy_id_buf[n_legacy], src, L);
            s_legacy_id_buf[n_legacy][L] = '\0';
            s_legacy_id_ptrs[n_legacy] = s_legacy_id_buf[n_legacy];
            n_legacy++;
        }
    }
    s_cfg.legacy_node_ids = (n_legacy > 0) ? s_legacy_id_ptrs : NULL;
    s_cfg.legacy_node_ids_count = n_legacy;
    derive_ids_and_topics();
    s_boot_us = esp_timer_get_time();
    s_last_diag_us = 0;
}

void ha_mqtt_start(void) {
    if (s_client) return;

    esp_mqtt_client_config_t mc = {
        .broker.address.uri = s_cfg.broker_uri ? s_cfg.broker_uri : "mqtt://192.168.1.23",
        .credentials = {
            .client_id = s_devid,
            .username = s_cfg.username,
            .authentication.password = s_cfg.password
        },
        .session = {
            .keepalive = 15,
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

    /* Keep HA availability fresh to avoid transient 'unavailable' due to Wi-Fi flaps */
    pub(s_topic_status, "online", 1, 1);

    /* If we detect distance for the first time, advertise the entity dynamically */
    if (distance_mm >= 0 && !s_distance_advertised) {
        s_cfg.distance_supported = true;
        publish_discovery_all();
        s_distance_advertised = true;
    }

    if ((s_cfg.distance_supported || s_distance_advertised) && distance_mm >= 0) {
        // smoothing buffer update (mm)
        if (s_smooth_win < 1) {
            s_smooth_win = 1;
        }
        if (s_smooth_win > 10) {
            s_smooth_win = 10;
        }
        s_smooth_ring[s_smooth_head] = distance_mm;
        s_smooth_head = (s_smooth_head + 1) % s_smooth_win;
        if (s_smooth_count < s_smooth_win) s_smooth_count++;
        int sum = 0; for (int i=0;i<s_smooth_count;i++) sum += s_smooth_ring[i];
        int avg_mm = sum / (s_smooth_count ? s_smooth_count : 1);
        char buf[16];
        float cm = avg_mm / 10.0f;
        snprintf(buf, sizeof(buf), "%.1f", cm);
        pub(s_topic_distance_cm, buf, 0, 0);

        // zones (consider distance alone sufficient to trigger zone active)
        int cur_cm = (avg_mm + 5) / 10; // round mm to cm
        for (int i = 0; i < 3; ++i) {
            int on = (cur_cm >= s_zone_min_cm[i] && cur_cm <= s_zone_max_cm[i]) ? 1 : 0;
            if (on != s_zone_last_on[i]) {
                pub(s_topic_zone_active[i], on?"ON":"OFF", 0, 0);
                s_zone_last_on[i] = on;
            }
        }
    }

    publish_periodic_diag_if_due();
}

void ha_mqtt_publish_rssi_now(void) {
    s_last_diag_us = 0; // force send on next call
    publish_periodic_diag_if_due();
}

void ha_mqtt_resend_discovery(void) {
    if (!s_connected) return;
    if (s_discovery_suppressed) { ESP_LOGW(TAG, "HA discovery suppressed; skipping resend"); return; }
    publish_discovery_all();
    publish_attrs_once();
    publish_periodic_diag_if_due();
}

void ha_mqtt_diag_publish_out(int raw, int active, int present) {
    if (!s_connected) return;
    pub(s_topic_diag_out_raw, raw?"ON":"OFF", 0, 0);
    pub(s_topic_diag_out_active, active?"ON":"OFF", 0, 0);
    pub(s_topic_diag_out_present, present?"ON":"OFF", 0, 0);
}

void ha_mqtt_diag_publish_uart(int alive, int baud) {
    if (!s_connected) return;
    char b[16]; snprintf(b, sizeof(b), "%d", baud);
    pub(s_topic_diag_uart_alive, alive?"ON":"OFF", 0, 0);
    pub(s_topic_diag_uart_baud, b, 0, 0);
}

void ha_mqtt_publish_dir_approach(int on) {
    if (!s_connected) return;
    pub(s_topic_dir_approach, on ? "ON" : "OFF", 1, 0);
}
void ha_mqtt_publish_dir_away(int on) {
    if (!s_connected) return;
    pub(s_topic_dir_away, on ? "ON" : "OFF", 1, 0);
}
