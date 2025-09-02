#include "ld2420.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_idf_version.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "sensor_info.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <ctype.h>

static const char *TAG = "ld2420";

/* =========================
 *   Static state
 * ========================= */
static ld2420_cfg_t s_cfg;
static TaskHandle_t s_task = NULL;
static QueueHandle_t s_gpio_evtq = NULL;  // OT2 edge queue

static volatile bool     s_present = false;
static volatile int      s_last_distance_mm = -1;
static volatile int64_t  s_last_distance_us = 0;

static volatile bool     s_uart_active = false;
static volatile int      s_uart_baud = 0;
static volatile int64_t  s_uart_last_byte_us = 0;
static bool              s_warned_no_uart = false;
static volatile int      s_uart_cfg_baud = 0;   // last configured baud on the peripheral
static gpio_num_t        s_uart_tx_pin_cur;
static gpio_num_t        s_uart_rx_pin_cur;
static bool              s_fixed_mode = false;  // stick to fixed baud, no periodic autobaud
static int               s_fixed_baud = 0;      // fixed baud value when s_fixed_mode
static bool              s_loaded_ot2_ah = false; // loaded OT2 polarity from NVS
static int               s_ot2_pull_override = -1; // -1=auto, 0=off, 1=on
/* OT2 pulse-activity assist (helps when line is PWM/noisy). If raw edges exceed threshold
   within a short window, treat as activity and refresh hold_on/presence. */
static int               s_ot2_pulse_win_ms = 250;   // window in ms
static int               s_ot2_pulse_thresh = 3;     // edges per window to count as active
static int               s_ot2_pulse_edges = 0;
static int64_t           s_ot2_pulse_start_us = 0;

/* OT2 debounce/hold */
static bool     s_ot2_last = false;     // last *active* sample
static bool     s_ot2_stable = false;   // debounced active state
static int64_t  s_ot2_edge_us = 0;      // last change timestamp
static int64_t  s_last_high_us = 0;     // last time active=true
/* OT2 raw monitor */
static int      s_ot2_mon_prev_raw = -1;    // previous raw level
static int      s_ot2_mon_edges = 0;        // present edges since last log
static int      s_ot2_mon_raw_changes = 0;  // raw level changes since last log
static int64_t  s_ot2_mon_last_us = 0;      // last monitor log time

/* Publishing cadence limiter */
static int64_t  s_last_pub_us = 0;
/* UART RX monitor */
static int      s_rx_mon_prev = -1;
static int      s_rx_mon_edges = 0;
static int64_t  s_rx_mon_last_us = 0;
static int64_t  s_baud_lock_until_us = 0;  // do not re-autobaud until after this time
static int      s_baud_locked_baud  = 0;
static int64_t  s_last_keepalive_us = 0;   // last time we sent a stream-enable keepalive
static bool     s_proto_sample_logged = false; // logged a header/framing sample
static bool     s_fw_logged = false;           // logged a firmware version candidate
static char     s_fw_version[48] = {0};        // cached version text if found (large to appease -Wformat-truncation)
static QueueHandle_t s_uart_evtq = NULL;       // UART driver event queue
static uint8_t  s_ring[1024];                  // simple ring buffer for framed parsing
static int      s_ring_len = 0;
/* removed unused QueueSetHandle_t s_evtset to avoid warnings treated as errors */

/* =========================
 *   Utilities
 * ========================= */
static inline int64_t now_us(void) { return esp_timer_get_time(); }
static inline bool pin_is_valid(gpio_num_t pin) { return (pin >= 0) && (pin < GPIO_NUM_MAX); }

/* =========================
 *   UART helpers
 * ========================= */
static void send_boot_seq(void)
{
    const char *ascii = s_cfg.boot_tx_ascii;
    const char *hex   = s_cfg.boot_tx_hex;
    int reps = s_cfg.boot_tx_repeat > 0 ? s_cfg.boot_tx_repeat : 1;
    int dly  = s_cfg.boot_tx_delay_ms >= 0 ? s_cfg.boot_tx_delay_ms : 20;

    // If nothing configured, try a small built-in candidate list
    const char *auto_ascii_list[] = {
        "UARTRPT=1\r\n",
        "report=1\r\n",
        NULL
    };
    const char *auto_hex_list[] = {
        /* Common stream-enabling nibbles some boards expect */
        "AA 55 01 00",
        /* HLK-LD2420 vendor frames (FDFCFBFA ... 04030201) */
        /* Open command mode */
        "FD FC FB FA 04 00 FF 00 01 00 04 03 02 01",
        /* Read version number */
        "FD FC FB FA 02 00 00 00 04 03 02 01",
        /* Some firmware variants require an INIT-like command before responses */
        /* Matches Arduino LD2420_CMD_INIT: FDFCFBFA 08 00 12 00 00 00 64 00 00 00 04 03 02 01 */
        "FD FC FB FA 08 00 12 00 00 00 64 00 00 00 04 03 02 01",
        /* Close command mode */
        "FD FC FB FA 02 00 FE 00 04 03 02 01",
        NULL
    };

    bool use_auto = (!ascii || !ascii[0]) && (!hex || !hex[0]);
    if (use_auto) {
        ESP_LOGI(TAG, "UART boot: sending auto probe commands to enable stream");
    }

    for (int r = 0; r < reps; ++r) {
        if (ascii && ascii[0]) {
            uart_write_bytes(s_cfg.uart_num, (const uint8_t *)ascii, strlen(ascii));
        } else if (use_auto) {
            for (int i = 0; auto_ascii_list[i]; ++i) {
                const char *s = auto_ascii_list[i];
                uart_write_bytes(s_cfg.uart_num, (const uint8_t *)s, strlen(s));
                if (dly > 0) vTaskDelay(pdMS_TO_TICKS(dly));
            }
        }

        if (hex && hex[0]) {
            const char *s = hex;
            while (*s) {
                while (*s == ' ' || *s == ',' || *s == '\t') s++;
                if (!*s) break;
                char *end = NULL;
                long v = strtol(s, &end, 16);
                if (end == s) break; // no progress
                uint8_t b = (uint8_t)(v & 0xFF);
                uart_write_bytes(s_cfg.uart_num, &b, 1);
                s = end;
            }
        } else if (use_auto) {
            for (int i = 0; auto_hex_list[i]; ++i) {
                const char *s = auto_hex_list[i];
                while (*s) {
                    while (*s == ' ' || *s == ',' || *s == '\t') s++;
                    if (!*s) break;
                    char *end = NULL;
                    long v = strtol(s, &end, 16);
                    if (end == s) break;
                    uint8_t b = (uint8_t)(v & 0xFF);
                    uart_write_bytes(s_cfg.uart_num, &b, 1);
                    s = end;
                }
                if (dly > 0) vTaskDelay(pdMS_TO_TICKS(dly));
            }
        }

        if (dly > 0) vTaskDelay(pdMS_TO_TICKS(dly));
    }
    uart_wait_tx_done(s_cfg.uart_num, pdMS_TO_TICKS(50));
}
static esp_err_t uart_setup(int baud)
{
    const uart_config_t uc = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION_MAJOR >= 5
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    ESP_RETURN_ON_ERROR(uart_param_config(s_cfg.uart_num, &uc), TAG, "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(s_cfg.uart_num, s_uart_tx_pin_cur, s_uart_rx_pin_cur,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin");

    if (pin_is_valid(s_uart_rx_pin_cur)) {
        gpio_pullup_en(s_uart_rx_pin_cur);  // define RX idle on the actual mapped pin
    }
    // Make RX more responsive to short bursts
    uart_set_rx_full_threshold(s_cfg.uart_num, 1);
    uart_set_rx_timeout(s_cfg.uart_num, 2);
    s_uart_cfg_baud = baud;
    ESP_LOGI(TAG, "UART%u mapped: TX=GPIO%d RX=GPIO%d baud=%d",
             (unsigned)s_cfg.uart_num, (int)s_uart_tx_pin_cur, (int)s_uart_rx_pin_cur, baud);
    return ESP_OK;
}

static bool uart_seems_alive(int check_ms)
{
    const int64_t end_us = now_us() + (int64_t)check_ms * 1000;
    uint8_t tmp[128];
    size_t total = 0;
    while (now_us() < end_us) {
        int n = uart_read_bytes(s_cfg.uart_num, tmp, sizeof(tmp), pdMS_TO_TICKS(10));
        if (n > 0) {
            total += n;
            s_uart_last_byte_us = now_us();
            if (total >= 4) break; // accept short bursts
        }
    }
    return total >= 1; // consider alive on any traffic
}

static void uart_autobaud(void)
{
    if (s_fixed_mode) {
        // Respect fixed mode: keep configured baud, no scanning
        if (s_fixed_baud <= 0) s_fixed_baud = s_cfg.baud_primary;
        uart_setup(s_fixed_baud);
        s_uart_active = false; // until we sniff data from replies
        s_uart_baud = s_fixed_baud;
        s_warned_no_uart = false;
        s_baud_locked_baud = s_fixed_baud;
        s_baud_lock_until_us = now_us() + 3600LL*1000000LL; // 1 hour lock
        ESP_LOGI(TAG, "UART fixed at %d baud (fixed mode)", s_fixed_baud);
        return;
    }
    s_uart_active = false;
    s_uart_baud = 0;

    ESP_LOGD(TAG, "UART probing primary=%d", s_cfg.baud_primary);
    if (uart_setup(s_cfg.baud_primary) == ESP_OK) {
        send_boot_seq();
        if (uart_seems_alive(900)) {
            s_uart_active = true;
            s_uart_baud = s_cfg.baud_primary;
            s_warned_no_uart = false;
            s_baud_locked_baud = s_uart_baud;
            s_baud_lock_until_us = now_us() + 600000000LL; // 10 minutes
            ESP_LOGI(TAG, "UART active at %d baud", s_uart_baud);
            return;
        }
    }

    ESP_LOGD(TAG, "UART probing fallback=%d", s_cfg.baud_fallback);
    if (uart_setup(s_cfg.baud_fallback) == ESP_OK) {
        send_boot_seq();
        if (uart_seems_alive(1200)) {
            s_uart_active = true;
            s_uart_baud = s_cfg.baud_fallback;
            s_warned_no_uart = false;
            s_baud_locked_baud = s_uart_baud;
            s_baud_lock_until_us = now_us() + 600000000LL;
            ESP_LOGI(TAG, "UART active at %d baud (fallback)", s_uart_baud);
            return;
        }
    }

    /* Try a few additional common baud rates */
    const int extra_bauds[] = {230400, 460800, 57600, 38400, 19200, 9600};
    for (size_t i = 0; i < sizeof(extra_bauds)/sizeof(extra_bauds[0]); ++i) {
        if (uart_setup(extra_bauds[i]) == ESP_OK) {
            send_boot_seq();
            if (uart_seems_alive(700)) {
                s_uart_active = true;
                s_uart_baud = extra_bauds[i];
                s_warned_no_uart = false;
                s_baud_locked_baud = s_uart_baud;
                s_baud_lock_until_us = now_us() + 600000000LL;
                ESP_LOGI(TAG, "UART active at %d baud (extra)", s_uart_baud);
                return;
            }
        }
    }

    /* Probe with swapped pins in case RX/TX were crossed in wiring */
    if (pin_is_valid(s_cfg.uart_tx) && pin_is_valid(s_cfg.uart_rx)) {
        gpio_num_t tx0 = s_uart_tx_pin_cur, rx0 = s_uart_rx_pin_cur;
        s_uart_tx_pin_cur = rx0; s_uart_rx_pin_cur = tx0;
        ESP_LOGW(TAG, "UART probing with swapped pins: TX=GPIO%d RX=GPIO%d", (int)s_uart_tx_pin_cur, (int)s_uart_rx_pin_cur);
        if (uart_setup(s_cfg.baud_primary) == ESP_OK) {
            send_boot_seq();
            if (uart_seems_alive(900)) {
                s_uart_active = true;
                s_uart_baud = s_cfg.baud_primary;
                s_warned_no_uart = false;
                s_baud_locked_baud = s_uart_baud;
                s_baud_lock_until_us = now_us() + 600000000LL;
                ESP_LOGW(TAG, "UART became active with swapped pins at %d baud", s_uart_baud);
                return;
            }
        }
        if (uart_setup(s_cfg.baud_fallback) == ESP_OK) {
            send_boot_seq();
            if (uart_seems_alive(1200)) {
                s_uart_active = true;
                s_uart_baud = s_cfg.baud_fallback;
                s_warned_no_uart = false;
                s_baud_locked_baud = s_uart_baud;
                s_baud_lock_until_us = now_us() + 600000000LL;
                ESP_LOGW(TAG, "UART became active with swapped pins at %d baud (fallback)", s_uart_baud);
                return;
            }
        }
        /* restore original mapping */
        s_uart_tx_pin_cur = tx0; s_uart_rx_pin_cur = rx0;
        uart_setup(s_cfg.baud_primary);
    }

    ESP_LOGW(TAG, "UART autobaud failed (no traffic at %d or %d)",
             s_cfg.baud_primary, s_cfg.baud_fallback);
}

/* =========================
 *   Presence + publishing
 * ========================= */
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

/* =========================
 *   OT2 diagnostics
 * ========================= */
static void ot2_pull_diagnostic(gpio_num_t pin)
{
    if (!pin_is_valid(pin)) return;

    // Ensure input mode
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    // Baseline with pulls disabled
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

    ESP_LOGD(TAG, "OT2 diag: baseline=%d pulldown=%d pullup=%d", base, with_dn, with_up);
}

/* =========================
 *   OT2 interrupt
 * ========================= */
static void IRAM_ATTR ot2_isr(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if (s_gpio_evtq) {
        xQueueSendFromISR(s_gpio_evtq, &gpio_num, NULL);
    }
}

static void update_presence_from_ot2(bool active, int raw_lvl, int64_t t)
{
    if (active != s_ot2_last) {
        s_ot2_last = active;
        s_ot2_edge_us = t;
    }

    const bool stable = (t - s_ot2_edge_us) >= (int64_t)s_cfg.debounce_ms * 1000;
    if (stable) s_ot2_stable = s_ot2_last;

    if (s_ot2_stable) s_last_high_us = t;

    bool new_present = s_present;
    if (s_ot2_stable) {
        new_present = true;
    } else if ((t - s_last_high_us) > (int64_t)s_cfg.hold_on_ms * 1000) {
        new_present = false;
    }

    if (new_present != s_present) {
        s_present = new_present;
        ESP_LOGI(TAG, "Presence %s (lvl=%d, active=%d, polarity=%s)",
                 s_present ? "ON" : "OFF",
                 raw_lvl, active ? 1 : 0,
                 s_cfg.ot2_active_high ? "AH" : "AL");
        publish(true, s_present);
    }
}

static void apply_ot2_pullcfg(void)
{
    if (!pin_is_valid(s_cfg.ot2_gpio)) return;
    gpio_pulldown_dis(s_cfg.ot2_gpio);
    if (s_ot2_pull_override == 1) {
        gpio_pullup_en(s_cfg.ot2_gpio);
        ESP_LOGI(TAG, "OT2: internal pull-up forced ON");
    } else if (s_ot2_pull_override == 0) {
        gpio_pullup_dis(s_cfg.ot2_gpio);
        ESP_LOGI(TAG, "OT2: internal pull-up forced OFF");
    } else {
        if (s_cfg.ot2_active_high) {
            gpio_pullup_dis(s_cfg.ot2_gpio);
            ESP_LOGI(TAG, "OT2: pull-up AUTO (off; AH)");
        } else {
            gpio_pullup_en(s_cfg.ot2_gpio);
            ESP_LOGI(TAG, "OT2: pull-up AUTO (on; AL)");
        }
    }
}

static void force_recalc_presence_now(void)
{
    if (!pin_is_valid(s_cfg.ot2_gpio)) return;
    const int64_t t = now_us();
    int lvl = gpio_get_level(s_cfg.ot2_gpio);
    bool active = s_cfg.ot2_active_high ? (lvl == 1) : (lvl == 0);
    // Make debouncer consider it stable immediately
    s_ot2_last = active;
    s_ot2_edge_us = t - (int64_t)s_cfg.debounce_ms * 1000;
    s_ot2_stable = active;
    if (active) s_last_high_us = t;
    bool new_present = s_ot2_stable ? true : ((t - s_last_high_us) <= (int64_t)s_cfg.hold_on_ms * 1000);
    if (new_present != s_present) {
        s_present = new_present;
        ESP_LOGI(TAG, "Presence %s (lvl=%d, active=%d, polarity=%s)",
                 s_present ? "ON" : "OFF", lvl, active ? 1 : 0,
                 s_cfg.ot2_active_high ? "AH" : "AL");
    }
    publish(true, s_present);
}

/* =========================
 *   UART scraper (ASCII + binary heuristics)
 * ========================= */
static bool try_parse_known_frame(const uint8_t *buf, int len, int *mm_out)
{
    // Heuristic: frames starting with 0xFF 0x8D or 0xFF 0x40 with distance at +8 (LE)
    for (int i = 0; i + 10 < len; ++i) {
        if (buf[i] == 0xFF && (buf[i+1] == 0x8D || buf[i+1] == 0x40)) {
            int mm = (int)buf[i+8] | ((int)buf[i+9] << 8);
            if (mm >= 5 && mm <= 6000) {
                *mm_out = mm;
                return true;
            }
        }
    }
    return false;
}

static void parse_ld2420_blob(const uint8_t *buf, int len, int64_t t_now)
{
    /* Use ASCII ON/OFF hints regardless of OT2 wiring to avoid getting stuck
       when OT2 is idle/miswired or not configured on the module. */
    for (int i = 0; i < len; ++i) {
        if (i + 1 < len && buf[i] == 'O' && buf[i+1] == 'N') {
            if (!s_present) {
                s_present = true;
                s_last_high_us = t_now;
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

    int last_mm = -1;
    /* Try known binary frame heuristic first */
    int mm_known = -1;
    if (try_parse_known_frame(buf, len, &mm_known)) {
        s_last_distance_mm = mm_known;
        s_last_distance_us = t_now;
        publish(false, s_present);
        // continue to version/logging scan below
    }

    /* ASCII decimal heuristic (legacy text streams) */
    for (int i = 0; i < len; ) {
        if (isdigit((int)buf[i])) {
            int j = i, val = 0, digits = 0;
            while (j < len && isdigit((int)buf[j]) && digits < 6) {
                val = val * 10 + (buf[j] - '0');
                j++; digits++;
            }
            if (val >= 5 && val <= 6000) last_mm = val;
            i = j;
        } else {
            i++;
        }
    }
    /* Generic binary heuristic: scan 16-bit little-endian words for plausible mm.
       Many LD2420 firmwares emit binary frames where distance appears as
       a LE16 value in-range within the payload (e.g., ... BE 00 ... = 190mm). */
    for (int i = 0; i + 1 < len; ++i) {
        int val = (int)buf[i] | ((int)buf[i+1] << 8);
        if (val >= 5 && val <= 6000) {
            last_mm = val; // take the last plausible value in the chunk
        }
    }

    if (last_mm >= 0) {
        s_last_distance_mm = last_mm;
        s_last_distance_us = t_now;
        publish(false, s_present);
    }

    /* One-time protocol sample log: if we detect the structured header 0xFAFBFCFD,
       log up to 32 bytes starting at the header to help reverse-engineer fields. */
    if (!s_proto_sample_logged) {
        for (int i = 0; i + 4 < len; ++i) {
            if (buf[i] == 0xFA && buf[i+1] == 0xFB && buf[i+2] == 0xFC && buf[i+3] == 0xFD) {
                int m = (len - i) < 32 ? (len - i) : 32;
                char hex[3*32+1]; int k = 0;
                for (int j = 0; j < m; ++j) k += snprintf(hex+k, sizeof(hex)-k, "%02X ", buf[i+j]);
                ESP_LOGI(TAG, "LD2420 frame sample: %s%s", hex, (m < (len - i) ? "..." : ""));
                s_proto_sample_logged = true;
                break;
            }
        }
    }

    /* Firmware version candidate: scan for ASCII pattern d+.d+.d+ and log once */
    if (!s_fw_logged) {
        for (int i = 0; i < len; ++i) {
            // parse major.minor.patch with small digit limits to avoid false hits
            int p = i, maj=0, min=0, pat=0, d=0;
            while (p < len && isdigit((int)buf[p]) && d < 2) { maj = maj*10 + (buf[p]-'0'); p++; d++; }
            if (p >= len || buf[p] != '.' || d == 0) { continue; }
            p++;
            d = 0;
            while (p < len && isdigit((int)buf[p]) && d < 2) { min = min*10 + (buf[p]-'0'); p++; d++; }
            if (p >= len || buf[p] != '.' || d == 0) { continue; }
            p++;
            d = 0;
            while (p < len && isdigit((int)buf[p]) && d < 3) { pat = pat*10 + (buf[p]-'0'); p++; d++; }
            if (d == 0) { continue; }
            // sanity check version ranges
            if (maj >= 0 && maj <= 9 && min >= 0 && min <= 99 && pat >= 0 && pat <= 999) {
                // Use a large buffer; print as unsigned to avoid sign warnings
                unsigned umaj = (unsigned)maj, umin = (unsigned)min, upat = (unsigned)pat;
                snprintf(s_fw_version, sizeof(s_fw_version), "%u.%u.%u", umaj, umin, upat);
                ESP_LOGI(TAG, "LD2420 firmware version candidate: %s", s_fw_version);
                sensor_info_set_fw(s_fw_version);
                s_fw_logged = true;
                break;
            }
        }
    }
}

/* =========================
 *   Framed parser (FA FB FC FD ... 04 03 02 01)
 * ========================= */
static void rb_append(const uint8_t *data, int n)
{
    if (n <= 0) return;
    if (n > (int)sizeof(s_ring)) {
        // Only keep tail that fits
        data += (n - (int)sizeof(s_ring));
        n = sizeof(s_ring);
    }
    if (s_ring_len + n > (int)sizeof(s_ring)) {
        int drop = (s_ring_len + n) - (int)sizeof(s_ring);
        memmove(s_ring, s_ring + drop, s_ring_len - drop);
        s_ring_len -= drop;
    }
    memcpy(s_ring + s_ring_len, data, n);
    s_ring_len += n;
}

/* =========================
 *   Vendor command helpers (FD FC FB FA ... 04 03 02 01)
 * ========================= */
static void vframe_send(const uint8_t *payload, int payload_len)
{
    if (payload_len <= 0) return;
    uint8_t buf[4 + 2 + 256 + 4];
    if (payload_len > 256) return;
    int k = 0;
    buf[k++] = 0xFD; buf[k++] = 0xFC; buf[k++] = 0xFB; buf[k++] = 0xFA;
    buf[k++] = (uint8_t)(payload_len & 0xFF);
    buf[k++] = (uint8_t)((payload_len >> 8) & 0xFF);
    memcpy(buf + k, payload, payload_len); k += payload_len;
    buf[k++] = 0x04; buf[k++] = 0x03; buf[k++] = 0x02; buf[k++] = 0x01;
    uart_write_bytes(s_cfg.uart_num, buf, k);
    uart_wait_tx_done(s_cfg.uart_num, pdMS_TO_TICKS(20));
}

static void vcmd_u16_only(uint16_t cmd)
{
    uint8_t p[2];
    p[0] = (uint8_t)(cmd & 0xFF);
    p[1] = (uint8_t)((cmd >> 8) & 0xFF);
    vframe_send(p, 2);
}

static void vcmd_read_param(uint16_t pname)
{
    uint8_t p[4];
    p[0] = 0x08; p[1] = 0x00; // READ PARAMS
    p[2] = (uint8_t)(pname & 0xFF);
    p[3] = (uint8_t)((pname >> 8) & 0xFF);
    vframe_send(p, 4);
}

static void vcmd_set_param(uint16_t pname, uint32_t val)
{
    uint8_t p[8];
    p[0] = 0x07; p[1] = 0x00; // SET PARAMS
    p[2] = (uint8_t)(pname & 0xFF);
    p[3] = (uint8_t)((pname >> 8) & 0xFF);
    p[4] = (uint8_t)(val & 0xFF);
    p[5] = (uint8_t)((val >> 8) & 0xFF);
    p[6] = (uint8_t)((val >> 16) & 0xFF);
    p[7] = (uint8_t)((val >> 24) & 0xFF);
    vframe_send(p, 8);
}

static bool parse_u32(const char *s, uint32_t *out)
{
    if (!s || !*s) return false;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (end == s) return false;
    *out = (uint32_t)v;
    return true;
}

static bool parse_u16(const char *s, uint16_t *out)
{
    uint32_t v;
    if (!parse_u32(s, &v)) return false;
    if (v > 0xFFFF) return false;
    *out = (uint16_t)v;
    return true;
}

static uint16_t pname_from_tokens(const char *p0, const char *p1)
{
    if (!p0) return 0xFFFF;
    if (!strcasecmp(p0, "min")) return 0x0000;
    if (!strcasecmp(p0, "max")) return 0x0001;
    if (!strcasecmp(p0, "delay")) return 0x0004;
    if (!strncasecmp(p0, "trig", 4)) {
        uint16_t idx = 0;
        if (p1 && parse_u16(p1, &idx) && idx < 16) return (uint16_t)(0x0010 + idx);
        return 0xFFFF;
    }
    if (!strncasecmp(p0, "hold", 4)) {
        uint16_t idx = 0;
        if (p1 && parse_u16(p1, &idx) && idx < 16) return (uint16_t)(0x0020 + idx);
        return 0xFFFF;
    }
    return 0xFFFF;
}

void ld2420_mqtt_cmd(const char *payload, int len)
{
    if (!payload || len <= 0) return;
    // Copy to buffer and null-terminate
    char buf[96]; int n = len < (int)sizeof(buf)-1 ? len : (int)sizeof(buf)-1; memcpy(buf, payload, n); buf[n] = '\0';
    // Tokenize (space-separated)
    char *tok[5] = {0}; int nt=0; char *s = buf;
    while (*s && nt < 5) {
        while (*s && isspace((int)*s)) s++;
        if (!*s) break;
        tok[nt++] = s;
        while (*s && !isspace((int)*s)) s++;
        if (*s) { *s = '\0'; s++; }
    }
    if (nt == 0) return;
    if (!strcasecmp(tok[0], "open")) {
        vcmd_u16_only(0x00FF);
        ESP_LOGI(TAG, "LD2420 CMD: open mode");
        return;
    }
    if (!strcasecmp(tok[0], "close")) {
        vcmd_u16_only(0x00FE);
        ESP_LOGI(TAG, "LD2420 CMD: close mode");
        return;
    }
    if (!strcasecmp(tok[0], "reboot") || !strcasecmp(tok[0], "restart")) {
        vcmd_u16_only(0x0068);
        ESP_LOGI(TAG, "LD2420 CMD: reboot");
        return;
    }
    if (!strcasecmp(tok[0], "init")) {
        /* Arduino LD2420_CMD_INIT */
        uint8_t p[8] = {0x12,0x00,0x00,0x00,0x64,0x00,0x00,0x00};
        vframe_send(p, 8);
        ESP_LOGI(TAG, "LD2420 CMD: init");
        return;
    }
    if (!strcasecmp(tok[0], "factory") || !strcasecmp(tok[0], "factory_reset")) {
        /* Arduino LD2420_CMD_FACTORY_RESET (dangerous) */
        uint8_t p[4] = {0xA2,0x00,0x00,0x00};
        vframe_send(p, 4);
        ESP_LOGW(TAG, "LD2420 CMD: factory reset sent");
        return;
    }
    if (!strcasecmp(tok[0], "ot2") && nt >= 2) {
        if (nt >= 3 && !strcasecmp(tok[1], "pullup")) {
            if (!strcasecmp(tok[2], "on") || !strcasecmp(tok[2], "1")) {
                s_ot2_pull_override = 1;
            } else if (!strcasecmp(tok[2], "off") || !strcasecmp(tok[2], "0")) {
                s_ot2_pull_override = 0;
            } else {
                s_ot2_pull_override = -1; // auto
            }
            apply_ot2_pullcfg();
            force_recalc_presence_now();
            nvs_handle_t h;
            if (nvs_open("ps_ld2420", NVS_READWRITE, &h) == ESP_OK) {
                nvs_set_i8(h, "ot2_pu", (int8_t)s_ot2_pull_override);
                nvs_commit(h);
                nvs_close(h);
            }
            ESP_LOGI(TAG, "LD2420 CMD: OT2 pull-up %s", s_ot2_pull_override==1?"on": s_ot2_pull_override==0?"off":"auto");
            return;
        }
        if (nt >= 4 && !strcasecmp(tok[1], "pulse")) {
            if (!strcasecmp(tok[2], "win")) {
                uint32_t v=0;
                if (parse_u32(tok[3], &v)) {
                    if ((int)v < 50) {
                        v = 50;
                    }
                    if ((int)v > 2000) {
                        v = 2000;
                    }
                    s_ot2_pulse_win_ms = (int)v;
                    nvs_handle_t h;
                    if (nvs_open("ps_ld2420", NVS_READWRITE, &h) == ESP_OK) {
                        nvs_set_i32(h, "ot2_pwin", (int32_t)v);
                        nvs_commit(h);
                        nvs_close(h);
                    }
                    ESP_LOGI(TAG, "LD2420 CMD: OT2 pulse window = %u ms", (unsigned)v);
                }
                return;
            }
            if (!strcasecmp(tok[2], "thresh")) {
                uint32_t v=0;
                if (parse_u32(tok[3], &v)) {
                    if ((int)v < 1) {
                        v = 1;
                    }
                    if ((int)v > 50) {
                        v = 50;
                    }
                    s_ot2_pulse_thresh = (int)v;
                    nvs_handle_t h;
                    if (nvs_open("ps_ld2420", NVS_READWRITE, &h) == ESP_OK) {
                        nvs_set_i32(h, "ot2_ptsh", (int32_t)v);
                        nvs_commit(h);
                        nvs_close(h);
                    }
                    ESP_LOGI(TAG, "LD2420 CMD: OT2 pulse thresh = %u edges", (unsigned)v);
                }
                return;
            }
        }
        if (!pin_is_valid(s_cfg.ot2_gpio)) { ESP_LOGW(TAG, "LD2420 CMD: OT2 not wired"); return; }
        if (!strcasecmp(tok[1], "invert")) {
            s_cfg.ot2_active_high = !s_cfg.ot2_active_high;
        } else if (!strcasecmp(tok[1], "high") || !strcasecmp(tok[1], "ah")) {
            s_cfg.ot2_active_high = true;
        } else if (!strcasecmp(tok[1], "low") || !strcasecmp(tok[1], "al")) {
            s_cfg.ot2_active_high = false;
        } else {
            ESP_LOGW(TAG, "LD2420 CMD: ot2 <high|low|invert|pullup on|off|auto|pulse win <ms>|pulse thresh <n>>");
            return;
        }
        apply_ot2_pullcfg();
        force_recalc_presence_now();
        // persist to NVS
        nvs_handle_t h;
        if (nvs_open("ps_ld2420", NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_u8(h, "ot2_ah", (uint8_t)(s_cfg.ot2_active_high ? 1 : 0));
            nvs_commit(h);
            nvs_close(h);
        }
        ESP_LOGI(TAG, "LD2420 CMD: OT2 polarity set to %s", s_cfg.ot2_active_high?"active-high":"active-low");
        return;
    }
    if (!strcasecmp(tok[0], "version") || (nt>=2 && !strcasecmp(tok[0], "read") && !strcasecmp(tok[1], "version"))) {
        vcmd_u16_only(0x0000);
        ESP_LOGI(TAG, "LD2420 CMD: read version");
        return;
    }
    if (!strcasecmp(tok[0], "reautobaud") || !strcasecmp(tok[0], "scan")) {
        s_fixed_mode = false;
        s_baud_lock_until_us = 0;
        s_warned_no_uart = false;
        uart_autobaud();
        ESP_LOGI(TAG, "LD2420 CMD: reautobaud");
        return;
    }
    if (!strcasecmp(tok[0], "lockbaud") || !strcasecmp(tok[0], "fixed")) {
        s_fixed_mode = true;
        s_fixed_baud = (s_cfg.fixed_baud_rate > 0 ? s_cfg.fixed_baud_rate : s_cfg.baud_primary);
        uart_setup(s_fixed_baud);
        s_baud_locked_baud = s_fixed_baud;
        s_baud_lock_until_us = now_us() + 3600LL*1000000LL;
        ESP_LOGI(TAG, "LD2420 CMD: lock baud at %d", s_fixed_baud);
        return;
    }
    if (!strcasecmp(tok[0], "read") && nt >= 2) {
        uint16_t pn = pname_from_tokens(tok[1], nt>=3?tok[2]:NULL);
        if (pn != 0xFFFF) {
            vcmd_read_param(pn);
            ESP_LOGI(TAG, "LD2420 CMD: read param 0x%04X", pn);
        } else {
            ESP_LOGW(TAG, "LD2420 CMD: unknown read param");
        }
        return;
    }
    if (!strcasecmp(tok[0], "set") && nt >= 3) {
        uint16_t pn = pname_from_tokens(tok[1], (nt>=4?tok[2]:NULL));
        const char *vstr = (pn>=0x0010 && pn<=0x002F) ? (nt>=4?tok[3]:NULL) : tok[2];
        uint32_t val=0;
        if (pn!=0xFFFF && vstr && parse_u32(vstr, &val)) {
            vcmd_set_param(pn, val);
            ESP_LOGI(TAG, "LD2420 CMD: set 0x%04X = %u (0x%08X)", pn, (unsigned)val, (unsigned)val);
        } else {
            ESP_LOGW(TAG, "LD2420 CMD: bad set syntax");
        }
        return;
    }
    ESP_LOGW(TAG, "LD2420 CMD: unrecognized command");
}
static void try_parse_frames_from_ring(int64_t t_now)
{
    /* Accept both vendor-documented header FDFCFBFA and the reverse (FAFBFCFD) seen in some firmwares */
    const uint8_t H1[4] = {0xFD,0xFC,0xFB,0xFA};
    const uint8_t H2[4] = {0xFA,0xFB,0xFC,0xFD};
    const uint8_t T[4]  = {0x04,0x03,0x02,0x01};
    int pos = 0;
    while (pos + 8 <= s_ring_len) {
        // find header
        int h = -1;
        int which = 0; // 1 for H1, 2 for H2
        for (int i = pos; i + 3 < s_ring_len; ++i) {
            if (s_ring[i]==H1[0] && s_ring[i+1]==H1[1] && s_ring[i+2]==H1[2] && s_ring[i+3]==H1[3]) { h = i; which = 1; break; }
            if (s_ring[i]==H2[0] && s_ring[i+1]==H2[1] && s_ring[i+2]==H2[2] && s_ring[i+3]==H2[3]) { h = i; which = 2; break; }
        }
        if (h < 0) {
            // keep last 3 bytes to match next header
            if (s_ring_len > 3) {
                memmove(s_ring, s_ring + s_ring_len - 3, 3);
                s_ring_len = 3;
            }
            break;
        }
        // Attempt length-based framing: [H(4)] [LEN(2, LE)] [PAYLOAD LEN bytes] [T(4)]
        int f = -1;
        int framelen = -1;
        if (h + 6 <= s_ring_len) {
            int len = (int)s_ring[h+4] | ((int)s_ring[h+5] << 8);
            if (len >= 0 && len <= 900) {
                int expect_total = 4 /*H*/ + 2 /*LEN*/ + len + 4 /*T*/;
                if (h + expect_total <= s_ring_len) {
                    int tail = h + 4 + 2 + len;
                    if (s_ring[tail]==T[0] && s_ring[tail+1]==T[1] && s_ring[tail+2]==T[2] && s_ring[tail+3]==T[3]) {
                        f = tail;
                        framelen = expect_total;
                    }
                } else {
                    // wait for more bytes
                    if (h > 0) {
                        memmove(s_ring, s_ring + h, s_ring_len - h);
                        s_ring_len -= h;
                    }
                    break;
                }
            }
        }
        // Fallback: search footer if length framing not conclusive
        if (f < 0) {
            for (int j = h + 4; j + 3 < s_ring_len; ++j) {
                if (s_ring[j]==T[0] && s_ring[j+1]==T[1] && s_ring[j+2]==T[2] && s_ring[j+3]==T[3]) { f = j; break; }
            }
            if (f < 0) {
                // wait for more bytes
                if (h > 0) {
                    memmove(s_ring, s_ring + h, s_ring_len - h);
                    s_ring_len -= h;
                }
                break;
            }
            framelen = (f + 4) - h;
        }
        if (!s_proto_sample_logged) {
            int m = framelen < 32 ? framelen : 32;
            char hex[3*32+1]; int k=0;
            for (int j=0;j<m;++j) k += snprintf(hex+k, sizeof(hex)-k, "%02X ", s_ring[h+j]);
            ESP_LOGI(TAG, "LD2420 frame sample (H%s): %s%s", which==1?"FDFCFBFA":"FAFBFCFD", hex, (framelen>m?"...":""));
            s_proto_sample_logged = true;
        }
        // Best-effort distance extraction: try offset +8 (LE16). If LEN was present, payload starts at h+6
        if (h + 10 <= s_ring_len) {
            int mm = (int)s_ring[h+8] | ((int)s_ring[h+9] << 8);
            if (mm >= 5 && mm <= 6000) {
                s_last_distance_mm = mm;
                s_last_distance_us = t_now;
                publish(false, s_present);
            }
        }
        // consume up to end of frame
        int consume = f + 4;
        memmove(s_ring, s_ring + consume, s_ring_len - consume);
        s_ring_len -= consume;
        pos = 0; // restart from beginning (buffer now shifted)
    }
}

/* =========================
 *   Worker task
 * ========================= */
static void ld2420_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LD2420 task");

    /* UART driver install */
    const int rx_buf = 2048;
    const int tx_buf = 0;
    /* Initialize current UART pin mapping from config */
    s_uart_tx_pin_cur = s_cfg.uart_tx;
    s_uart_rx_pin_cur = s_cfg.uart_rx;

    ESP_ERROR_CHECK(uart_driver_install(s_cfg.uart_num, rx_buf, tx_buf, 20, &s_uart_evtq, 0));
    // Initialize fixed/auto baud strategy
    if (s_fixed_mode) {
        if (s_fixed_baud <= 0) s_fixed_baud = s_cfg.baud_primary;
        ESP_ERROR_CHECK(uart_setup(s_fixed_baud));
        uart_flush_input(s_cfg.uart_num);
        s_uart_last_byte_us = now_us();
        s_uart_active = false; // will turn true when we sniff responses
        s_uart_baud = s_fixed_baud;
        s_baud_locked_baud = s_fixed_baud;
        s_baud_lock_until_us = now_us() + 3600LL*1000000LL;
        ESP_LOGI(TAG, "UART fixed init at %d baud", s_fixed_baud);
    } else {
        ESP_ERROR_CHECK(uart_setup(s_cfg.baud_primary));
        uart_flush_input(s_cfg.uart_num);
        s_uart_last_byte_us = now_us();
        uart_autobaud();
    }

    /* OT2 setup (interrupt on both edges, plus polling fallback) */
    if (pin_is_valid(s_cfg.ot2_gpio)) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << s_cfg.ot2_gpio),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_ANYEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        // Setup ISR queue + handler
        if (!s_gpio_evtq) {
            s_gpio_evtq = xQueueCreate(8, sizeof(uint32_t));
        }
        esp_err_t isrres = gpio_install_isr_service(0);
        if (isrres != ESP_OK && isrres != ESP_ERR_INVALID_STATE) {
            ESP_ERROR_CHECK(isrres);
        }
        ESP_ERROR_CHECK(gpio_isr_handler_add(s_cfg.ot2_gpio, ot2_isr, (void *)(uint32_t)s_cfg.ot2_gpio));

        // Quick safe diagnostic to check if the line responds to internal pulls
        ot2_pull_diagnostic(s_cfg.ot2_gpio);
    }

    /* Initial state & publish */
    s_last_pub_us = 0;
    s_last_distance_mm = -1;
    s_last_distance_us = 0;
    s_last_high_us = 0;
    s_ot2_last = false;
    s_ot2_stable = false;
    s_ot2_mon_prev_raw = -1;
    s_ot2_mon_edges = 0;
    s_ot2_mon_raw_changes = 0;
    s_ot2_mon_last_us = now_us();
    s_ot2_pulse_edges = 0;
    s_ot2_pulse_start_us = now_us();
    s_rx_mon_prev = -1;
    s_rx_mon_edges = 0;
    s_rx_mon_last_us = now_us();

    if (pin_is_valid(s_cfg.ot2_gpio)) {
        int idle = gpio_get_level(s_cfg.ot2_gpio);
        ESP_LOGI(TAG, "OT2 configured polarity: %s (idle=%d)",
                 s_cfg.ot2_active_high ? "active-high" : "active-low", idle);

        /* Many LD2420 boards drive OT2 as an open-drain/collector output that pulls LOW
           when active and relies on an external pull-up when inactive. If we are configured
           as active-low, help the line with an internal pull-up. Otherwise, leave pulls off
           to avoid fighting external resistors. */
        gpio_pulldown_dis(s_cfg.ot2_gpio);
        if (!s_cfg.ot2_active_high) {
            gpio_pullup_en(s_cfg.ot2_gpio);
            ESP_LOGI(TAG, "OT2: internal pull-up enabled (active-low)");
        } else {
            gpio_pullup_dis(s_cfg.ot2_gpio);
        }

        bool active = s_cfg.ot2_active_high ? (idle == 1) : (idle == 0);
        const int64_t t = now_us();
        s_ot2_last = active;
        s_ot2_edge_us = t - (int64_t)s_cfg.debounce_ms * 1000;
        s_ot2_stable = active;
        s_present = active;
        if (active) s_last_high_us = t;

        ESP_LOGI(TAG, "Initial presence: %s (lvl=%d, active=%d)",
                 s_present ? "ON" : "OFF", idle, active ? 1 : 0);
        publish(true, s_present);
    } else {
        s_present = false;
        ESP_LOGI(TAG, "OT2 not wired; starting with presence OFF");
        publish(true, s_present);
    }

    /* Loop */
    const TickType_t tick = pdMS_TO_TICKS(s_cfg.sample_ms);
    uint8_t buf[512];
    int64_t last_auto_us = 0;

    while (1) {
        /* Wait a bit; also wake up early on OT2 edge */
        uint32_t gpio_num = 0;
        if (s_gpio_evtq) {
            if (xQueueReceive(s_gpio_evtq, &gpio_num, tick)) {
                const int64_t t = now_us();
                int lvl = gpio_get_level(s_cfg.ot2_gpio);
                bool active = s_cfg.ot2_active_high ? (lvl == 1) : (lvl == 0);
                update_presence_from_ot2(active, lvl, t);
            }
        } else {
            vTaskDelay(tick);
        }
        const int64_t tnow = now_us();

        /* Poll OT2 (covers missed edges) */
        if (pin_is_valid(s_cfg.ot2_gpio)) {
            int lvl = gpio_get_level(s_cfg.ot2_gpio);
            if (s_ot2_mon_prev_raw < 0) s_ot2_mon_prev_raw = lvl;
            if (lvl != s_ot2_mon_prev_raw) {
                s_ot2_mon_raw_changes++;
                s_ot2_mon_prev_raw = lvl;
                // Count pulse edges for activity assist
                s_ot2_pulse_edges++;
            }

            bool active = s_cfg.ot2_active_high ? (lvl == 1) : (lvl == 0);
            bool prev_present = s_present;
            update_presence_from_ot2(active, lvl, tnow);
            if (s_present != prev_present) s_ot2_mon_edges++;

            /* Periodic raw monitor log (~2s) */
            if ((tnow - s_ot2_mon_last_us) > 2000000LL) {
                s_ot2_mon_last_us = tnow;
                ESP_LOGD(TAG, "OT2 mon: lvl=%d active=%d stable=%d present=%d edges=%d rawchg=%d", \
                         lvl, active ? 1 : 0, s_ot2_stable ? 1 : 0, s_present ? 1 : 0, \
                         s_ot2_mon_edges, s_ot2_mon_raw_changes);
                s_ot2_mon_edges = 0;
                s_ot2_mon_raw_changes = 0;
            }

            /* Pulse-activity assist: within window, if edges exceed threshold, refresh presence */
            int64_t pwin_us = (int64_t)s_ot2_pulse_win_ms * 1000;
            if (pwin_us < 50000) pwin_us = 50000; // clamp >=50ms
            if ((tnow - s_ot2_pulse_start_us) >= pwin_us) {
                if (s_ot2_pulse_edges >= s_ot2_pulse_thresh) {
                    // Consider this activity: keep presence on and refresh last_high
                    if (!s_present) {
                        s_present = true;
                        ESP_LOGD(TAG, "OT2 pulse assist: presence ON (edges=%d/%d win=%dms)",
                                 s_ot2_pulse_edges, s_ot2_pulse_thresh, s_ot2_pulse_win_ms);
                        publish(true, s_present);
                    }
                    s_last_high_us = tnow;
                }
                s_ot2_pulse_edges = 0;
                s_ot2_pulse_start_us = tnow;
            }
        }

        /* UART scraping (event-driven preferred) */
        bool had_uart_evt = false;
        if (s_uart_evtq) {
            uart_event_t e;
            if (xQueueReceive(s_uart_evtq, &e, 0)) {
                had_uart_evt = true;
                if (e.type == UART_DATA && e.size > 0) {
                    int toread = e.size > (int)sizeof(buf) ? (int)sizeof(buf) : e.size;
                    int n = uart_read_bytes(s_cfg.uart_num, buf, toread, 0);
                    if (n > 0) {
                        s_uart_last_byte_us = tnow;
                        s_warned_no_uart = false;
                        rb_append(buf, n);
                        try_parse_frames_from_ring(tnow);
                        parse_ld2420_blob(buf, n, tnow);
                    }
                } else if (e.type == UART_FIFO_OVF || e.type == UART_BUFFER_FULL) {
                    uart_flush_input(s_cfg.uart_num);
                }
            }
        }
        if (!had_uart_evt) {
            if (s_uart_active) {
                int n = uart_read_bytes(s_cfg.uart_num, buf, sizeof(buf), 0);
                if (n > 0) {
                    s_uart_last_byte_us = tnow;
                    s_warned_no_uart = false;
                    rb_append(buf, n);
                    try_parse_frames_from_ring(tnow);
                    parse_ld2420_blob(buf, n, tnow);
                } else if ((tnow - s_uart_last_byte_us) > 15000000LL) {
                    s_uart_active = false;
                }
            } else {
                // Even when inactive, try to sniff bytes at the currently configured baud
                int n = uart_read_bytes(s_cfg.uart_num, buf, sizeof(buf), 0);
                if (n > 0) {
                    s_uart_last_byte_us = tnow;
                    s_uart_active = true;
                    s_uart_baud = s_uart_cfg_baud;
                    s_warned_no_uart = false;
                    s_baud_locked_baud = s_uart_baud;
                    s_baud_lock_until_us = now_us() + 600000000LL;
                    int m = n < 12 ? n : 12;
                    char hex[3*12+1]; int k=0;
                    for (int i=0;i<m;i++) k += snprintf(hex+k, sizeof(hex)-k, "%02X ", buf[i]);
                    ESP_LOGI(TAG, "UART became active at %d (sniffed %dB: %s%s)",
                             s_uart_baud, n, hex, (n>m?"...":""));
                    rb_append(buf, n);
                    try_parse_frames_from_ring(tnow);
                    parse_ld2420_blob(buf, n, tnow);
                }
            }
        }

        /* RX line monitor: sample mapped RX pin as GPIO to see toggles */
        if (pin_is_valid(s_uart_rx_pin_cur)) {
            int lvl = gpio_get_level(s_uart_rx_pin_cur);
            if (s_rx_mon_prev < 0) s_rx_mon_prev = lvl;
            if (lvl != s_rx_mon_prev) {
                s_rx_mon_edges++;
                s_rx_mon_prev = lvl;
            }
            if ((tnow - s_rx_mon_last_us) > 2000000LL) {
                ESP_LOGD(TAG, "UART RX mon: pin=%d lvl=%d edges=%d baud=%d active=%d",
                         (int)s_uart_rx_pin_cur, lvl, s_rx_mon_edges, s_uart_cfg_baud, s_uart_active ? 1 : 0);
                s_rx_mon_edges = 0;
                s_rx_mon_last_us = tnow;
            }
        }

        /* Retry autobaud every ~5s if inactive; warn once until traffic returns */
        if (!s_fixed_mode && !s_uart_active && (tnow - last_auto_us) > 5000000LL) {
            last_auto_us = tnow;
            if (s_baud_lock_until_us == 0 || tnow >= s_baud_lock_until_us) {
                uart_autobaud();
            }
            if (!s_uart_active && !s_warned_no_uart) {
                ESP_LOGW(TAG, "No UART traffic detected at %d or %d baud",
                         s_cfg.baud_primary, s_cfg.baud_fallback);
                s_warned_no_uart = true;
            }
        }

        /* Keepalive: some firmwares stop streaming unless periodically poked. */
        if ((tnow - s_uart_last_byte_us) > 10000000LL && (tnow - s_last_keepalive_us) > 10000000LL) { // 10s
            if (s_uart_cfg_baud > 0) {
                send_boot_seq();
                s_last_keepalive_us = tnow;
                ESP_LOGD(TAG, "UART keepalive sent");
            }
        }

        /* Periodic heartbeat publish (rate-limited inside) */
        publish(false, s_present);
    }
}

/* =========================
 *   Public API
 * ========================= */
esp_err_t ld2420_init(const ld2420_cfg_t *cfg)
{
    if (!cfg || !cfg->cb) return ESP_ERR_INVALID_ARG;

    s_cfg = *cfg;

    // Initialize fixed-baud strategy from config
    s_fixed_mode = s_cfg.uart_fixed_baud;
    s_fixed_baud = (s_cfg.fixed_baud_rate > 0 ? s_cfg.fixed_baud_rate : s_cfg.baud_primary);

    if (s_cfg.sample_ms <= 0)            s_cfg.sample_ms = 50;
    if (s_cfg.debounce_ms <= 0)          s_cfg.debounce_ms = 150;
    if (s_cfg.hold_on_ms <= 0)           s_cfg.hold_on_ms = 3000;
    if (s_cfg.min_pub_interval_ms <= 0)  s_cfg.min_pub_interval_ms = 10000;
    if (s_cfg.stale_after_ms <= 0)       s_cfg.stale_after_ms = 2000;

    if (s_cfg.baud_primary   <= 0) s_cfg.baud_primary   = 115200;  // FW ≥ 1.5.3
    if (s_cfg.baud_fallback  <= 0) s_cfg.baud_fallback  = 256000;  // legacy
    if (s_cfg.fixed_baud_rate <= 0) s_cfg.fixed_baud_rate = s_cfg.baud_primary;

    if (!pin_is_valid(s_cfg.uart_tx) || !pin_is_valid(s_cfg.uart_rx)) {
        ESP_LOGE(TAG, "Invalid UART pins");
        return ESP_ERR_INVALID_ARG;
    }

    s_present = false;
    s_last_distance_mm = -1;
    s_last_distance_us = 0;

    s_uart_active = false;
    s_uart_baud = 0;
    s_uart_last_byte_us = 0;
    s_warned_no_uart = false;

    /* Load persisted tuning if available */
    nvs_handle_t h; int32_t v;
    if (nvs_open("ps_ld2420", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_i32(h, "debounce_ms", &v) == ESP_OK) {
            ld2420_set_debounce_ms((int)v);
            ESP_LOGI(TAG, "Loaded debounce_ms=%d from NVS", (int)v);
        }
        if (nvs_get_i32(h, "hold_on_ms", &v) == ESP_OK) {
            ld2420_set_hold_on_ms((int)v);
            ESP_LOGI(TAG, "Loaded hold_on_ms=%d from NVS", (int)v);
        }
        uint8_t b;
        if (nvs_get_u8(h, "ot2_ah", &b) == ESP_OK) {
            s_cfg.ot2_active_high = (b ? true : false);
            s_loaded_ot2_ah = true;
            ESP_LOGI(TAG, "Loaded ot2_active_high=%d from NVS", (int)(b?1:0));
        }
        int8_t puv;
        if (nvs_get_i8(h, "ot2_pu", &puv) == ESP_OK) {
            if (puv < -1) {
                puv = -1;
            }
            if (puv > 1) {
                puv = 1;
            }
            s_ot2_pull_override = (int)puv;
            ESP_LOGI(TAG, "Loaded ot2_pull_override=%d from NVS", (int)puv);
        }
        int32_t pwin;
        if (nvs_get_i32(h, "ot2_pwin", &pwin) == ESP_OK) {
            if (pwin < 50) {
                pwin = 50;
            }
            if (pwin > 2000) {
                pwin = 2000;
            }
            s_ot2_pulse_win_ms = (int)pwin;
            ESP_LOGI(TAG, "Loaded ot2_pulse_win_ms=%d from NVS", (int)pwin);
        }
        int32_t pts;
        if (nvs_get_i32(h, "ot2_ptsh", &pts) == ESP_OK) {
            if (pts < 1) {
                pts = 1;
            }
            if (pts > 50) {
                pts = 50;
            }
            s_ot2_pulse_thresh = (int)pts;
            ESP_LOGI(TAG, "Loaded ot2_pulse_thresh=%d from NVS", (int)pts);
        }
        nvs_close(h);
    }

    return ESP_OK;
}

esp_err_t ld2420_start(void)
{
    if (s_task) return ESP_OK;
    BaseType_t ok = xTaskCreate(ld2420_task, "ld2420_task", 4096, NULL, 4, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void ld2420_stop(void)
{
    if (!s_task) return;
    vTaskDelete(s_task);
    s_task = NULL;
    if (pin_is_valid(s_cfg.ot2_gpio)) {
        gpio_isr_handler_remove(s_cfg.ot2_gpio);
    }
    if (s_gpio_evtq) {
        vQueueDelete(s_gpio_evtq);
        s_gpio_evtq = NULL;
    }
    uart_driver_delete(s_cfg.uart_num);
}

bool ld2420_get_present(void)          { return s_present; }
int  ld2420_get_last_distance_mm(void) { return s_last_distance_mm; }
bool ld2420_uart_active(void)          { return s_uart_active; }
int  ld2420_active_baud(void)          { return s_uart_baud; }

void ld2420_set_debounce_ms(int v) {
    if (v < 50) {
        v = 50;
    }
    if (v > 3000) {
        v = 3000;
    }
    s_cfg.debounce_ms = v;
    ESP_LOGI(TAG, "Set debounce_ms=%d", v);
    nvs_handle_t h;
    if (nvs_open("ps_ld2420", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "debounce_ms", v);
        nvs_commit(h);
        nvs_close(h);
    }
}
void ld2420_set_hold_on_ms(int v) {
    if (v < 500) {
        v = 500;
    }
    if (v > 15000) {
        v = 15000;
    }
    s_cfg.hold_on_ms = v;
    ESP_LOGI(TAG, "Set hold_on_ms=%d", v);
    nvs_handle_t h;
    if (nvs_open("ps_ld2420", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "hold_on_ms", v);
        nvs_commit(h);
        nvs_close(h);
    }
}
int  ld2420_get_debounce_ms(void) { return s_cfg.debounce_ms; }
int  ld2420_get_hold_on_ms(void)  { return s_cfg.hold_on_ms; }
