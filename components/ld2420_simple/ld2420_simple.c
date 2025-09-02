#include "ld2420_simple.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_idf_version.h"

static const char *TAG = "ld2420_simple";

static ld2420_simple_cfg_t s_cfg;
static TaskHandle_t s_task = NULL;
static volatile bool    s_present = false;
static volatile int     s_last_mm = -1;
static volatile int64_t s_last_detect_us = 0;
static bool s_ot2_last_active = false;
static bool s_ot2_stable = false;
static int64_t s_ot2_edge_us = 0;

static inline int64_t now_us(void) { return esp_timer_get_time(); }
static inline bool pin_is_valid(gpio_num_t pin) { return (pin >= 0) && (pin < GPIO_NUM_MAX); }

static void send_boot_seq(void)
{
    const char *ascii = s_cfg.boot_tx_ascii;
    const char *hex   = s_cfg.boot_tx_hex;
    int reps = s_cfg.boot_tx_repeat > 0 ? s_cfg.boot_tx_repeat : 2;
    int dly  = s_cfg.boot_tx_delay_ms > 0 ? s_cfg.boot_tx_delay_ms : 20;

    for (int r = 0; r < reps; ++r) {
        if (ascii && ascii[0]) {
            uart_write_bytes(s_cfg.uart_num, ascii, strlen(ascii));
        }
        if (hex && hex[0]) {
            const char *s = hex;
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
        }
        if (dly > 0) vTaskDelay(pdMS_TO_TICKS(dly));
    }
    uart_wait_tx_done(s_cfg.uart_num, pdMS_TO_TICKS(50));
}

static void flush_line(const char *line, int len, int64_t tnow)
{
    if (!line || len <= 0) return;
    // trim CR/LF and spaces
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n' || isspace((int)line[len-1]))) len--;
    if (len <= 0) return;
    // Presence tokens
    if ((len >= 2 && !strncmp(line, "ON", 2)) || (len >= 3 && !strncmp(line, "On", 2))) {
        s_present = true; s_last_detect_us = tnow; if (s_cfg.cb) s_cfg.cb(true, s_last_mm); return;
    }
    if (len >= 3 && !strncmp(line, "OFF", 3)) {
        s_present = false; if (s_cfg.cb) s_cfg.cb(false, -1); return;
    }
    // Range prefix
    if (len >= 6 && !strncmp(line, "Range ", 6)) {
        int val = 0; int i = 6; int digits=0;
        while (i < len && isdigit((int)line[i]) && digits < 6) { val = val*10 + (line[i]-'0'); i++; digits++; }
        if (digits > 0 && val >= 0 && val <= 6000) {
            s_last_mm = val * 10; s_last_detect_us = tnow; if (!s_present && s_cfg.cb) s_cfg.cb(true, s_last_mm); s_present = true; if (s_cfg.cb) s_cfg.cb(true, s_last_mm); return;
        }
    }
    // Generic decimal-in-line as cm
    int val = 0, digits = 0; for (int i=0;i<len;i++){ if (isdigit((int)line[i])) { val = val*10 + (line[i]-'0'); digits++; if (digits>5) break; } else if (digits>0) break; }
    if (digits>0 && val >= 0 && val <= 6000) { s_last_mm = val*10; s_last_detect_us = tnow; if (!s_present && s_cfg.cb) s_cfg.cb(true, s_last_mm); s_present = true; if (s_cfg.cb) s_cfg.cb(true, s_last_mm); }
}

static void parse_chunk(const uint8_t *buf, int len, int64_t tnow)
{
    static char line[128]; static int L = 0;
    for (int i = 0; i < len; ++i) {
        char c = (char)buf[i];
        if (c == '\n' || c == '\r') {
            if (L > 0) { flush_line(line, L, tnow); L = 0; }
        } else {
            if (L < (int)sizeof(line)-1) line[L++] = c;
            // Opportunistic scan inside partial line for tokens
            if (L >= 2 && line[L-2]=='O' && line[L-1]=='N') { s_present = true; s_last_detect_us = tnow; if (s_cfg.cb) s_cfg.cb(true, s_last_mm); }
            if (L >= 3 && line[L-3]=='O' && line[L-2]=='F' && line[L-1]=='F') { s_present = false; if (s_cfg.cb) s_cfg.cb(false, -1); }
        }
    }
}

static void task_fn(void *arg)
{
    ESP_LOGI(TAG, "Starting LD2420 simple task @%d baud", s_cfg.baud);

    // UART setup
    const uart_config_t uc = {
        .baud_rate  = s_cfg.baud > 0 ? s_cfg.baud : 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION_MAJOR >= 5
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    ESP_ERROR_CHECK(uart_param_config(s_cfg.uart_num, &uc));
    ESP_ERROR_CHECK(uart_set_pin(s_cfg.uart_num, s_cfg.uart_tx, s_cfg.uart_rx,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    if (pin_is_valid(s_cfg.uart_rx)) {
        gpio_pullup_en(s_cfg.uart_rx);
    }

    // Install driver with simple RX buffer first
    const int rx_buf = 2048;
    ESP_ERROR_CHECK(uart_driver_install(s_cfg.uart_num, rx_buf, 0, 0, NULL, 0));
    uart_flush_input(s_cfg.uart_num);
    // Small RX threshold so we get data quickly
    uart_set_rx_full_threshold(s_cfg.uart_num, 1);
    uart_set_rx_timeout(s_cfg.uart_num, 2);

    // Optional OT2 setup
    if (s_cfg.ot2_gpio >= 0 && s_cfg.ot2_gpio < GPIO_NUM_MAX) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << s_cfg.ot2_gpio),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = s_cfg.ot2_active_high ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        int idle = gpio_get_level(s_cfg.ot2_gpio);
        bool active = s_cfg.ot2_active_high ? (idle == 1) : (idle == 0);
        s_ot2_last_active = active;
        s_ot2_edge_us = now_us() - (int64_t)(s_cfg.debounce_ms > 0 ? s_cfg.debounce_ms : 100) * 1000;
        s_ot2_stable = active;
        if (active) s_last_detect_us = now_us();
        ESP_LOGI(TAG, "OT2 enabled: pin=%d polarity=%s idle=%d", (int)s_cfg.ot2_gpio,
                 s_cfg.ot2_active_high?"AH":"AL", idle);
    } else {
        ESP_LOGI(TAG, "OT2 disabled in simple driver");
    }

    // Optional boot sequence
    if ((s_cfg.boot_tx_ascii && s_cfg.boot_tx_ascii[0]) || (s_cfg.boot_tx_hex && s_cfg.boot_tx_hex[0])) {
        send_boot_seq();
    } else {
        // Built-in nudge: INIT frame and ASCII report toggle
        ld2420_simple_cfg_t tmp = s_cfg;
        tmp.boot_tx_ascii = "UARTRPT=1\r\n";
        tmp.boot_tx_hex   = "FD FC FB FA 08 00 12 00 00 00 64 00 00 00 04 03 02 01";
        tmp.boot_tx_repeat= 2; tmp.boot_tx_delay_ms = 20;
        s_cfg = tmp; send_boot_seq(); s_cfg = tmp; // keep values around
    }

    // Initial publish
    if (s_cfg.cb) s_cfg.cb(false, -1);

    const TickType_t tick = pdMS_TO_TICKS(s_cfg.sample_ms > 0 ? s_cfg.sample_ms : 50);
    uint8_t buf[512];

    while (1) {
        vTaskDelay(tick);
        int64_t tnow = now_us();

        int n = uart_read_bytes(s_cfg.uart_num, buf, sizeof(buf), 0);
        if (n > 0) {
            int m = n < 16 ? n : 16;
            char hex[3*16+1]; int k=0;
            for (int i=0;i<m;i++) k += snprintf(hex+k, sizeof(hex)-k, "%02X ", buf[i]);
            ESP_LOGD(TAG, "UART rx %dB: %s%s", n, hex, (n>m?"...":""));
            parse_chunk(buf, n, tnow);
        }

        // OT2 polling + debounce
        if (s_cfg.ot2_gpio >= 0 && s_cfg.ot2_gpio < GPIO_NUM_MAX) {
            int lvl = gpio_get_level(s_cfg.ot2_gpio);
            bool active_now = s_cfg.ot2_active_high ? (lvl == 1) : (lvl == 0);
            if (active_now != s_ot2_last_active) {
                s_ot2_last_active = active_now;
                s_ot2_edge_us = tnow;
            }
            int db = (s_cfg.debounce_ms > 0 ? s_cfg.debounce_ms : 100);
            if ((tnow - s_ot2_edge_us) >= (int64_t)db * 1000) {
                s_ot2_stable = s_ot2_last_active;
            }
            if (s_ot2_stable) {
                s_last_detect_us = tnow;
            }
        }

        // Simple hold logic
        int hold_ms = (s_cfg.hold_ms > 0 ? s_cfg.hold_ms : 1000);
        bool new_present = s_present;
        // Any source (UART/OT2) updates s_last_detect_us when active
        if ((tnow - s_last_detect_us) <= (int64_t)hold_ms * 1000) {
            new_present = true;
        } else {
            new_present = false;
        }
        if (new_present != s_present) {
            s_present = new_present;
            if (s_cfg.cb) s_cfg.cb(s_present, s_present ? s_last_mm : -1);
        }
    }
}

esp_err_t ld2420_simple_init(const ld2420_simple_cfg_t *cfg)
{
    if (!cfg || !cfg->cb) return ESP_ERR_INVALID_ARG;
    s_cfg = *cfg;
    s_present = false;
    s_last_mm = -1;
    s_last_detect_us = 0;
    return ESP_OK;
}

esp_err_t ld2420_simple_start(void)
{
    if (s_task) return ESP_OK;
    BaseType_t ok = xTaskCreate(task_fn, "ld2420_simple", 3072, NULL, 4, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void ld2420_simple_stop(void)
{
    if (!s_task) return;
    vTaskDelete(s_task);
    s_task = NULL;
    uart_driver_delete(s_cfg.uart_num);
}

bool ld2420_simple_get_present(void) { return s_present; }
int  ld2420_simple_get_last_distance_mm(void) { return s_last_mm; }
