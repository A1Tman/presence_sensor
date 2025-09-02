#include "ld2411.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "ha_mqtt.h"
#include "sensor_info.h"
#include "esp_check.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "ld2411";

static ld2411_cfg_t s_cfg;
static TaskHandle_t s_task = NULL;
static QueueHandle_t s_gpio_evtq = NULL;

static volatile bool     s_present = false;
static volatile int      s_last_distance_mm = -1;
static volatile int64_t  s_last_distance_us = 0;

static volatile bool     s_uart_active = false; // reserved for future UART parsing
static volatile int      s_uart_baud = 0;

static int64_t           s_last_pub_us = 0;
static bool              s_use_uart_presence = false;
static bool              s_out_pullup_enabled = false;
static int               s_presence_source = 0; // 0=out,1=uart,2=combined,3=distance
static int               s_dist_thresh_cm = 150;
static bool              s_dist_presence_enable = false;
/* no explicit uart_presence_flag used now */

static inline int64_t now_us(void) { return esp_timer_get_time(); }
static inline bool pin_is_valid(gpio_num_t pin) { return (pin >= 0) && (pin < GPIO_NUM_MAX); }
/* Forward declare vars used before definition */
static int64_t s_last_high_us;

/* ----------------- UART helpers ----------------- */
static QueueHandle_t s_uart_evtq = NULL;
static int64_t s_uart_last_byte_us = 0;
static bool s_warned_no_uart = false;

static void send_boot_seq(void)
{
    const char *ascii = s_cfg.boot_tx_ascii;
    const char *hex   = s_cfg.boot_tx_hex;
    int reps = s_cfg.boot_tx_repeat > 0 ? s_cfg.boot_tx_repeat : 1;
    int dly  = s_cfg.boot_tx_delay_ms >= 0 ? s_cfg.boot_tx_delay_ms : 20;

    for (int r = 0; r < reps; ++r) {
        if (ascii && ascii[0]) {
            uart_write_bytes(s_cfg.uart_num, ascii, strlen(ascii));
            if (dly > 0) vTaskDelay(pdMS_TO_TICKS(dly));
        }
        if (hex && hex[0]) {
            const char *p = hex; char *end = NULL;
            while (*p) {
                while (*p==' '||*p=='\t'||*p==',') p++;
                if (!*p) break;
                long v = strtol(p, &end, 16);
                if (end==p) {
                    break;
                }
                uint8_t b = (uint8_t)(v & 0xFF);
                uart_write_bytes(s_cfg.uart_num, &b, 1);
                p = end;
            }
            if (dly > 0) vTaskDelay(pdMS_TO_TICKS(dly));
        }
    }
    uart_wait_tx_done(s_cfg.uart_num, pdMS_TO_TICKS(50));
}

static esp_err_t uart_setup(int baud)
{
    uart_config_t uc = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION_MAJOR >= 5
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    ESP_RETURN_ON_ERROR(uart_param_config(s_cfg.uart_num, &uc), TAG, "uart_param_config");
    /* Only map pins that are valid; leave others unchanged to avoid driving external signals. */
    int tx_pin = pin_is_valid(s_cfg.uart_tx) ? (int)s_cfg.uart_tx : UART_PIN_NO_CHANGE;
    int rx_pin = pin_is_valid(s_cfg.uart_rx) ? (int)s_cfg.uart_rx : UART_PIN_NO_CHANGE;
    ESP_RETURN_ON_ERROR(uart_set_pin(s_cfg.uart_num, tx_pin, rx_pin,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "uart_set_pin");
    uart_set_rx_full_threshold(s_cfg.uart_num, 1);
    uart_set_rx_timeout(s_cfg.uart_num, 2);
    return ESP_OK;
}

static bool uart_seems_alive(int ms)
{
    const int64_t end = now_us() + ms*1000LL; uint8_t tmp[128]; size_t total=0;
    while (now_us() < end) {
        int n = uart_read_bytes(s_cfg.uart_num, tmp, sizeof(tmp), pdMS_TO_TICKS(10));
        if (n > 0) { total += n; s_uart_last_byte_us = now_us(); if (total>=4) break; }
    }
    return total > 0;
}

static void uart_autobaud(void)
{
    s_uart_active = false; s_uart_baud = 0;
    if (uart_setup(s_cfg.baud_primary) == ESP_OK) {
        send_boot_seq();
        if (uart_seems_alive(800)) { s_uart_active=true; s_uart_baud=s_cfg.baud_primary; s_warned_no_uart=false; ESP_LOGI(TAG, "UART active at %d", s_uart_baud); return; }
    }
    if (uart_setup(s_cfg.baud_fallback) == ESP_OK) {
        send_boot_seq();
        if (uart_seems_alive(1000)) { s_uart_active=true; s_uart_baud=s_cfg.baud_fallback; s_warned_no_uart=false; ESP_LOGI(TAG, "UART active at %d (fallback)", s_uart_baud); return; }
    }
}

/* Forward decl for publish, referenced from parse_uart_blob */
static void publish(bool force, bool present_now);

/*
 * UART scraping helper retained for future use on LD2411.
 * Marked unused to avoid compiler warnings when direction pins are used instead.
 */
static void __attribute__((unused)) parse_uart_blob(const uint8_t *buf, int len, int64_t tnow)
{
    /* ASCII presence hints (optional) */
    if (s_use_uart_presence) {
        for (int i = 0; i < len; ++i) {
            if (i + 1 < len && buf[i] == 'O' && buf[i+1] == 'N') {
                if (!s_present) {
                    s_present = true;
                    s_last_high_us = tnow;
                    publish(true, s_present);
                } else {
                    publish(false, s_present);
                }
            } else if (i + 2 < len && buf[i] == 'O' && buf[i+1] == 'F' && buf[i+2] == 'F') {
                if (s_present) {
                    s_present = false;
                    publish(true, s_present);
                } else {
                    publish(false, s_present);
                }
            }
        }
    }

    int last_mm = -1;
    // ASCII decimal heuristic
    for (int i=0;i<len;) {
        if (buf[i]>='0' && buf[i]<='9') {
            int j=i, val=0, digits=0;
            while (j<len && buf[j]>='0' && buf[j]<='9' && digits<6) { val = val*10 + (buf[j]-'0'); j++; digits++; }
            if (val>=5 && val<=6000) last_mm = val;
            i=j;
        } else i++;
    }
    // Generic LE16 scan
    for (int i=0;i+1<len;i++) {
        int v = (int)buf[i] | ((int)buf[i+1]<<8);
        if (v>=5 && v<=6000) last_mm = v;
    }
    if (last_mm >= 0) {
        s_last_distance_mm = last_mm;
        s_last_distance_us = tnow;
        publish(false, s_present);
    }
    // Firmware version candidate: d+.d+.d+
    for (int i=0;i<len;i++) {
        int p=i, maj=0,min=0,pat=0,d=0;
        while (p<len && (buf[p]>='0'&&buf[p]<='9') && d<2) { maj=maj*10+(buf[p]-'0'); p++; d++; }
        if (p>=len || buf[p] != '.' || d==0) {
            continue;
        }
        p++; d=0;
        while (p<len && (buf[p]>='0'&&buf[p]<='9') && d<2) { min=min*10+(buf[p]-'0'); p++; d++; }
        if (p>=len || buf[p] != '.' || d==0) {
            continue;
        }
        p++; d=0;
        while (p<len && (buf[p]>='0'&&buf[p]<='9') && d<3) { pat=pat*10+(buf[p]-'0'); p++; d++; }
        if (d==0) continue;
        if (maj<=9 && min<=99 && pat<=999) {
            char ver[48]; snprintf(ver, sizeof(ver), "%u.%u.%u", (unsigned)maj,(unsigned)min,(unsigned)pat);
            sensor_info_set_fw(ver);
            break;
        }
    }
}

/* ----------------- NVS helpers ----------------- */
static void nvs_save_i32(const char *ns, const char *key, int32_t val)
{
    nvs_handle_t h;
    if (nvs_open(ns, NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_i32(h, key, val) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}
static bool nvs_load_i32(const char *ns, const char *key, int32_t *out)
{
    nvs_handle_t h; int32_t v; esp_err_t err;
    if ((err = nvs_open(ns, NVS_READONLY, &h)) != ESP_OK) return false;
    err = nvs_get_i32(h, key, &v);
    nvs_close(h);
    if (err == ESP_OK) { *out = v; return true; }
    return false;
}

/* ----------------- Presence publish ----------------- */
static void publish(bool force, bool present_now)
{
    const int64_t t = now_us();
    const int64_t min_period = (int64_t)s_cfg.min_pub_interval_ms * 1000;
    if (!force && (t - s_last_pub_us) < min_period) return;
    s_last_pub_us = t;
    if (s_cfg.cb) {
        int d = s_last_distance_mm;
        if (d >= 0 && (t - s_last_distance_us) > (int64_t)s_cfg.stale_after_ms * 1000) d = -1;
        s_cfg.cb(present_now, d);
    }
}

/* ----------------- OUT (digital) handling ----------------- */
static bool     s_ot2_last = false;
static bool     s_ot2_stable = false;
static int64_t  s_ot2_edge_us = 0;
static int64_t  s_last_high_us = 0;
static int      s_ot2_mon_prev_raw = -1;
static int      s_ot2_mon_edges = 0;
static int      s_ot2_mon_raw_changes = 0;
static int64_t  s_ot2_mon_last_us = 0;
/* Direction pins (LD2411: RX/TX provide approach/away pulses) */
static int      s_dir_rx_prev_raw = -1;
static int      s_dir_tx_prev_raw = -1;
static bool     s_dir_approach_on = false;
static bool     s_dir_away_on = false;
static int64_t  s_dir_approach_on_us = 0;
static int64_t  s_dir_away_on_us = 0;
static int      s_dir_pulse_timeout_ms =
#ifdef LD2411_DIR_PULSE_TIMEOUT_MS
    LD2411_DIR_PULSE_TIMEOUT_MS
#else
    2500
#endif
    ; // auto-off guard in case pulse latches

/* Quick pull-diagnostic to help determine if the OUT line responds to internal pulls. */
static void ot2_pull_diagnostic(gpio_num_t pin)
{
    if (!pin_is_valid(pin)) return;
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    // Baseline (no pulls)
    gpio_pullup_dis(pin);
    gpio_pulldown_dis(pin);
    vTaskDelay(pdMS_TO_TICKS(2));
    int base = gpio_get_level(pin);
    // With pulldown
    gpio_pulldown_en(pin);
    vTaskDelay(pdMS_TO_TICKS(2));
    int with_dn = gpio_get_level(pin);
    gpio_pulldown_dis(pin);
    // With pullup
    gpio_pullup_en(pin);
    vTaskDelay(pdMS_TO_TICKS(2));
    int with_up = gpio_get_level(pin);
    gpio_pullup_dis(pin);
    ESP_LOGD(TAG, "OUT diag: baseline=%d pulldown=%d pullup=%d", base, with_dn, with_up);
}

static void IRAM_ATTR ot2_isr(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    (void)gpio_num;
    if (s_gpio_evtq) xQueueSendFromISR(s_gpio_evtq, &gpio_num, NULL);
}

static void update_presence_from_ot2(bool active, int raw_lvl, int64_t t)
{
    if (active != s_ot2_last) { s_ot2_last = active; s_ot2_edge_us = t; }
    const bool stable = (t - s_ot2_edge_us) >= (int64_t)s_cfg.debounce_ms * 1000;
    if (stable) s_ot2_stable = s_ot2_last;
    if (s_ot2_stable) s_last_high_us = t;

    bool new_present = s_present;
    if (s_ot2_stable) new_present = true;
    else if ((t - s_last_high_us) > (int64_t)s_cfg.hold_on_ms * 1000) new_present = false;

    if (new_present != s_present) {
        s_present = new_present;
        ESP_LOGI(TAG, "Presence %s (lvl=%d, active=%d, polarity=%s)",
                 s_present ? "ON" : "OFF", raw_lvl, active ? 1 : 0,
                 s_cfg.ot2_active_high ? "AH" : "AL");
        publish(true, s_present);
    }
}

/* ----------------- Worker task ----------------- */
static void ld2411_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LD2411 task (presence via OUT)");

    // Configure OUT (digital presence line)
    if (pin_is_valid(s_cfg.ot2_gpio)) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << s_cfg.ot2_gpio),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_ANYEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        if (!s_gpio_evtq) s_gpio_evtq = xQueueCreate(8, sizeof(uint32_t));
        esp_err_t isrres = gpio_install_isr_service(0);
        if (isrres != ESP_OK && isrres != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(isrres);
        ESP_ERROR_CHECK(gpio_isr_handler_add(s_cfg.ot2_gpio, ot2_isr, (void *)(uint32_t)s_cfg.ot2_gpio));
        gpio_intr_enable(s_cfg.ot2_gpio);

        // Pull config: many LD24xx boards use open-drain active-low OUT; LD2411 may be push-pull active-high per manual.
        ESP_LOGI(TAG, "OUT configured polarity: %s", s_cfg.ot2_active_high ? "active-high" : "active-low");
        s_out_pullup_enabled = (!s_cfg.ot2_active_high);
        gpio_pulldown_dis(s_cfg.ot2_gpio);
        if (s_out_pullup_enabled) {
            gpio_pullup_en(s_cfg.ot2_gpio);
            ESP_LOGI(TAG, "OUT: internal pull-up enabled (active-low)");
        } else {
            gpio_pullup_dis(s_cfg.ot2_gpio);
        }

        // Quick diagnostic of OUT responsiveness to internal pulls
        ot2_pull_diagnostic(s_cfg.ot2_gpio);

        // Initial state (after pull configuration & interrupt enable)
        int idle = gpio_get_level(s_cfg.ot2_gpio);
        bool active = s_cfg.ot2_active_high ? (idle == 1) : (idle == 0);
        const int64_t t = now_us();
        s_ot2_last = active;
        s_ot2_edge_us = t - (int64_t)s_cfg.debounce_ms * 1000;
        s_ot2_stable = active;
        s_present = active;
        if (active) s_last_high_us = t;
        s_ot2_mon_last_us = t;
        ESP_LOGI(TAG, "Initial presence: %s (lvl=%d, active=%d)", s_present ? "ON" : "OFF", idle, active ? 1 : 0);
        publish(true, s_present);
    } else {
        ESP_LOGW(TAG, "OUT pin not configured; presence will remain OFF unless UART heuristics are added");
        publish(true, false);
    }

    /* Configure LD2411 RX/TX pins as direction inputs (not UART). */
    if (pin_is_valid(s_cfg.uart_rx)) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << s_cfg.uart_rx),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_ANYEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        if (!s_gpio_evtq) s_gpio_evtq = xQueueCreate(8, sizeof(uint32_t));
        esp_err_t isrres = gpio_install_isr_service(0);
        if (isrres != ESP_OK && isrres != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(isrres);
        ESP_ERROR_CHECK(gpio_isr_handler_add(s_cfg.uart_rx, ot2_isr, (void *)(uint32_t)s_cfg.uart_rx));
        gpio_intr_enable(s_cfg.uart_rx);
        s_dir_rx_prev_raw = gpio_get_level(s_cfg.uart_rx);
    }
    if (pin_is_valid(s_cfg.uart_tx)) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << s_cfg.uart_tx),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_ANYEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        if (!s_gpio_evtq) s_gpio_evtq = xQueueCreate(8, sizeof(uint32_t));
        esp_err_t isrres = gpio_install_isr_service(0);
        if (isrres != ESP_OK && isrres != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(isrres);
        ESP_ERROR_CHECK(gpio_isr_handler_add(s_cfg.uart_tx, ot2_isr, (void *)(uint32_t)s_cfg.uart_tx));
        gpio_intr_enable(s_cfg.uart_tx);
        s_dir_tx_prev_raw = gpio_get_level(s_cfg.uart_tx);
    }
    ESP_LOGI(TAG, "Direction pins: RX=%d TX=%d (initial rx=%d tx=%d)",
             pin_is_valid(s_cfg.uart_rx)?(int)s_cfg.uart_rx:-1,
             pin_is_valid(s_cfg.uart_tx)?(int)s_cfg.uart_tx:-1,
             pin_is_valid(s_cfg.uart_rx)?gpio_get_level(s_cfg.uart_rx):-1,
             pin_is_valid(s_cfg.uart_tx)?gpio_get_level(s_cfg.uart_tx):-1);
    s_uart_evtq = NULL; // disable UART use on LD2411

    const TickType_t tick = pdMS_TO_TICKS(s_cfg.sample_ms > 0 ? s_cfg.sample_ms : 50);
    s_last_pub_us = 0;
    while (1) {
        uint32_t gpio_num = 0;
        if (s_gpio_evtq) {
            // Block for up to 'tick' waiting for an edge; this also yields CPU time
            if (xQueueReceive(s_gpio_evtq, &gpio_num, tick)) {
                const int64_t t = now_us();
                int lvl = gpio_get_level(s_cfg.ot2_gpio);
                bool active = s_cfg.ot2_active_high ? (lvl == 1) : (lvl == 0);
                int prev_present = s_present ? 1 : 0;
                update_presence_from_ot2(active, lvl, t);
                if (s_ot2_mon_prev_raw < 0) s_ot2_mon_prev_raw = lvl;
                if (lvl != s_ot2_mon_prev_raw) { s_ot2_mon_raw_changes++; s_ot2_mon_prev_raw = lvl; }
                if ((s_present ? 1 : 0) != prev_present) s_ot2_mon_edges++;

                // Evaluate direction on any edge
                if (pin_is_valid(s_cfg.uart_rx) || pin_is_valid(s_cfg.uart_tx)) {
                    int rx = pin_is_valid(s_cfg.uart_rx) ? gpio_get_level(s_cfg.uart_rx) : -1;
                    int tx = pin_is_valid(s_cfg.uart_tx) ? gpio_get_level(s_cfg.uart_tx) : -1;
                    bool approach = (rx == 1 && tx == 0);
                    bool away     = (tx == 1 && rx == 0);
                    if (rx >= 0 && rx != s_dir_rx_prev_raw) s_dir_rx_prev_raw = rx;
                    if (tx >= 0 && tx != s_dir_tx_prev_raw) s_dir_tx_prev_raw = tx;
                    if (approach != s_dir_approach_on) {
                        s_dir_approach_on = approach;
                        if (approach) s_dir_approach_on_us = now_us();
                        ha_mqtt_publish_dir_approach(approach?1:0);
                    }
                    if (away != s_dir_away_on) {
                        s_dir_away_on = away;
                        if (away) s_dir_away_on_us = now_us();
                        ha_mqtt_publish_dir_away(away?1:0);
                    }
                }
            }
        } else {
            // No ISR/queue configured (OUT not wired) — yield to avoid watchdog
            vTaskDelay(tick);
        }

        // Poll OUT to catch missed edges or when ISR isn't available
        if (pin_is_valid(s_cfg.ot2_gpio)) {
            const int64_t t = now_us();
            int lvl = gpio_get_level(s_cfg.ot2_gpio);
            bool active = s_cfg.ot2_active_high ? (lvl == 1) : (lvl == 0);
            int prev_present = s_present ? 1 : 0;
            update_presence_from_ot2(active, lvl, t);
            if (s_ot2_mon_prev_raw < 0) s_ot2_mon_prev_raw = lvl;
            if (lvl != s_ot2_mon_prev_raw) { s_ot2_mon_raw_changes++; s_ot2_mon_prev_raw = lvl; }
            if ((s_present ? 1 : 0) != prev_present) s_ot2_mon_edges++;
        }

        // Poll direction pins for robustness
        if (pin_is_valid(s_cfg.uart_rx) || pin_is_valid(s_cfg.uart_tx)) {
            int rx = pin_is_valid(s_cfg.uart_rx) ? gpio_get_level(s_cfg.uart_rx) : -1;
            int tx = pin_is_valid(s_cfg.uart_tx) ? gpio_get_level(s_cfg.uart_tx) : -1;
            bool approach = (rx == 1 && tx == 0);
            bool away     = (tx == 1 && rx == 0);
            if (rx >= 0 && rx != s_dir_rx_prev_raw) s_dir_rx_prev_raw = rx;
            if (tx >= 0 && tx != s_dir_tx_prev_raw) s_dir_tx_prev_raw = tx;
            if (approach != s_dir_approach_on) {
                s_dir_approach_on = approach;
                if (approach) s_dir_approach_on_us = now_us();
                ha_mqtt_publish_dir_approach(approach?1:0);
            }
            if (away != s_dir_away_on) {
                s_dir_away_on = away;
                if (away) s_dir_away_on_us = now_us();
                ha_mqtt_publish_dir_away(away?1:0);
            }
        }

        // Auto-off if a direction pulse appears latched high longer than timeout
        int64_t now_ts = now_us();
        if (s_dir_approach_on && (now_ts - s_dir_approach_on_us) > (int64_t)s_dir_pulse_timeout_ms*1000) {
            s_dir_approach_on = false; s_dir_approach_on_us = 0; ha_mqtt_publish_dir_approach(0);
        }
        if (s_dir_away_on && (now_ts - s_dir_away_on_us) > (int64_t)s_dir_pulse_timeout_ms*1000) {
            s_dir_away_on = false; s_dir_away_on_us = 0; ha_mqtt_publish_dir_away(0);
        }

        // Periodic OUT diagnostics (every ~2 seconds)
        if (now_ts - s_ot2_mon_last_us > 2000000LL) {
            s_ot2_mon_last_us = now_ts;
            int lvl = pin_is_valid(s_cfg.ot2_gpio) ? gpio_get_level(s_cfg.ot2_gpio) : -1;
            bool active = (lvl >= 0) ? (s_cfg.ot2_active_high ? (lvl == 1) : (lvl == 0)) : false;
            ESP_LOGD(TAG, "OUT mon: lvl=%d active=%d stable=%d present=%d edges=%d rawchg=%d",
                     lvl, active ? 1 : 0, s_ot2_stable ? 1 : 0, s_present ? 1 : 0,
                     s_ot2_mon_edges, s_ot2_mon_raw_changes);
            s_ot2_mon_edges = 0;
            s_ot2_mon_raw_changes = 0;
            // Publish diagnostics to HA
            if (lvl >= 0) {
                ha_mqtt_diag_publish_out(lvl, active?1:0, s_present?1:0);
            }
            ha_mqtt_diag_publish_uart(s_uart_active?1:0, s_uart_baud);
        }

        /* UART scraping disabled for LD2411 (direction pins used instead) */

        /* Distance-based presence contribution */
        if ((s_presence_source==2 || s_presence_source==3) && s_dist_presence_enable) {
            if (s_last_distance_mm >= 0 && (now_ts - s_last_distance_us) <= (int64_t)s_cfg.stale_after_ms*1000) {
                if (s_last_distance_mm <= s_dist_thresh_cm*10) {
                    s_last_high_us = now_ts;
                    if (s_presence_source==3 && !s_present) { s_present = true; publish(true, s_present); }
                }
            }
        }

        publish(false, s_present);
    }
}

/* ----------------- Public API ----------------- */
esp_err_t ld2411_init(const ld2411_cfg_t *cfg)
{
    if (!cfg || !cfg->cb) return ESP_ERR_INVALID_ARG;
    s_cfg = *cfg;
    if (s_cfg.sample_ms <= 0)            s_cfg.sample_ms = 50;
    if (s_cfg.debounce_ms <= 0)          s_cfg.debounce_ms = 120;
    if (s_cfg.hold_on_ms <= 0)           s_cfg.hold_on_ms = 1000;
    if (s_cfg.min_pub_interval_ms <= 0)  s_cfg.min_pub_interval_ms = 10000;
    if (s_cfg.stale_after_ms <= 0)       s_cfg.stale_after_ms = 2000;
    s_present = false;
    s_last_distance_mm = -1;
    s_last_distance_us = 0;
    s_uart_active = false;
    s_uart_baud = 0;
    /* Load persisted tuning if available */
    int32_t v;
    if (nvs_load_i32("ps_ld2411", "debounce_ms", &v)) {
        ld2411_set_debounce_ms((int)v);
        ESP_LOGI(TAG, "Loaded debounce_ms=%d from NVS", (int)v);
    }
    if (nvs_load_i32("ps_ld2411", "hold_on_ms", &v)) {
        ld2411_set_hold_on_ms((int)v);
        ESP_LOGI(TAG, "Loaded hold_on_ms=%d from NVS", (int)v);
    }
    if (nvs_load_i32("ps_ld2411", "presence_source", &v)) {
        ld2411_set_presence_source((int)v);
        ESP_LOGI(TAG, "Loaded presence_source=%d", (int)v);
    }
    if (nvs_load_i32("ps_ld2411", "dist_thresh_cm", &v)) {
        ld2411_set_distance_thresh_cm((int)v);
        ESP_LOGI(TAG, "Loaded dist_threshold=%d cm", (int)v);
    }
    if (nvs_load_i32("ps_ld2411", "dist_presence", &v)) {
        ld2411_set_distance_presence_enable(v!=0);
    }

    return ESP_OK;
}

esp_err_t ld2411_start(void)
{
    if (s_task) return ESP_OK;
    BaseType_t ok = xTaskCreate(ld2411_task, "ld2411_task", 4096, NULL, 4, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void ld2411_stop(void)
{
    if (!s_task) return;
    vTaskDelete(s_task);
    s_task = NULL;
    if (pin_is_valid(s_cfg.ot2_gpio)) gpio_isr_handler_remove(s_cfg.ot2_gpio);
    if (pin_is_valid(s_cfg.uart_rx)) gpio_isr_handler_remove(s_cfg.uart_rx);
    if (pin_is_valid(s_cfg.uart_tx)) gpio_isr_handler_remove(s_cfg.uart_tx);
    if (s_gpio_evtq) { vQueueDelete(s_gpio_evtq); s_gpio_evtq = NULL; }
    uart_driver_delete(s_cfg.uart_num);
}

bool ld2411_get_present(void)          { return s_present; }
int  ld2411_get_last_distance_mm(void) { return s_last_distance_mm; }
bool ld2411_uart_active(void)          { return s_uart_active; }
int  ld2411_active_baud(void)          { return s_uart_baud; }

void ld2411_set_debounce_ms(int v) {
    if (v < 50) {
        v = 50;
    }
    if (v > 3000) {
        v = 3000;
    }
    s_cfg.debounce_ms = v;
    ESP_LOGI(TAG, "Set debounce_ms=%d", v);
    nvs_save_i32("ps_ld2411", "debounce_ms", v);
}
void ld2411_set_hold_on_ms(int v) {
    if (v < 500) {
        v = 500;
    }
    if (v > 15000) {
        v = 15000;
    }
    s_cfg.hold_on_ms = v;
    ESP_LOGI(TAG, "Set hold_on_ms=%d", v);
    nvs_save_i32("ps_ld2411", "hold_on_ms", v);
}
int  ld2411_get_debounce_ms(void) { return s_cfg.debounce_ms; }
int  ld2411_get_hold_on_ms(void)  { return s_cfg.hold_on_ms; }

bool ld2411_get_out_active_high(void) { return s_cfg.ot2_active_high; }
void ld2411_set_out_active_high(bool on) { s_cfg.ot2_active_high = on; }
bool ld2411_get_out_pullup_enabled(void) { return s_out_pullup_enabled; }
void ld2411_set_out_pullup_enabled(bool on) {
    s_out_pullup_enabled = on;
    if (pin_is_valid(s_cfg.ot2_gpio)) {
        gpio_pulldown_dis(s_cfg.ot2_gpio);
        if (s_out_pullup_enabled) gpio_pullup_en(s_cfg.ot2_gpio); else gpio_pullup_dis(s_cfg.ot2_gpio);
    }
}
bool ld2411_get_uart_presence_enabled(void) { return s_use_uart_presence; }
void ld2411_set_uart_presence_enabled(bool on) { s_use_uart_presence = on; }

int  ld2411_get_presence_source(void) { return s_presence_source; }
void ld2411_set_presence_source(int src) {
    if (src < 0 || src > 3) src = 0;
    s_presence_source = src;
    nvs_save_i32("ps_ld2411", "presence_source", src);
}
int  ld2411_get_distance_thresh_cm(void) { return s_dist_thresh_cm; }
void ld2411_set_distance_thresh_cm(int v) {
    if (v < 10) {
        v = 10;
    }
    if (v > 600) {
        v = 600;
    }
    s_dist_thresh_cm = v;
    nvs_save_i32("ps_ld2411", "dist_thresh_cm", v);
}
bool ld2411_get_distance_presence_enable(void) { return s_dist_presence_enable; }
void ld2411_set_distance_presence_enable(bool on) {
    s_dist_presence_enable = on;
    nvs_save_i32("ps_ld2411", "dist_presence", on ? 1 : 0);
}

void ld2411_action_force_publish(void) { publish(true, s_present); }
void ld2411_action_reautobaud(void) { uart_autobaud(); }
void ld2411_action_reset_tuning(void) {
    ld2411_set_debounce_ms(120);
    ld2411_set_hold_on_ms(1000);
    ld2411_set_distance_thresh_cm(150);
    ld2411_set_distance_presence_enable(false);
    ld2411_set_presence_source(0);
}
